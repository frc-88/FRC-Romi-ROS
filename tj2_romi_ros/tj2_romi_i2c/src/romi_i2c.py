import math
import json
import time
import struct
import warnings
from Queue import Queue
from threading import Lock


class PinMode:
    kModeDigitalOut = 0
    kModeDigitalIn = 1
    kModeAnalogIn = 2
    kModePwm = 3
    modes = [kModeDigitalOut, kModeDigitalIn, kModeAnalogIn, kModePwm]

    @classmethod
    def is_mode(cls, mode):
        return mode in cls.modes


class Encoder:
    def __init__(self, start_ticks):
        self.prev_ticks = 0
        self.max_ticks = 0x7fff
        self.min_ticks = -0x8000
        self.max_dticks = 1000  # maximum possible instantenous change in encoder values
        self.delta_threshold = self.max_ticks - self.min_ticks - self.max_dticks
        self.overflow_count = 0
        self.start_ticks = start_ticks - self.min_ticks
    
    def update(self, ticks):
        dticks = ticks - self.prev_ticks
        self.prev_ticks = ticks
        if abs(dticks) > self.delta_threshold:
            # overflow has occurred
            # increment or decrement higher order bits
            if ticks > 0:
                self.overflow_count -= 1
            else:
                self.overflow_count += 1
    
    @property
    def ticks(self):
        ticks = (self.overflow_count << 16) | (self.prev_ticks - self.min_ticks)
        return ticks - self.start_ticks


HCSR04_OUT_OF_RANGE = 38000.0  # value ultrasonic sensor returns when out of range

class RomiI2C(object):
    def __init__(self, bus, sharedmem_path, wheel_diameter):
        self.bus = bus
        self.address = 0x14
        self.sharedmem_map = self.load_config(sharedmem_path)

        self.wheel_diameter = wheel_diameter  # m
        self.ticks_per_rotation = 1440.0
        self.ticks_to_m = math.pi * self.wheel_diameter / self.ticks_per_rotation

        self.io_modes = [PinMode.kModeDigitalOut, PinMode.kModeDigitalOut, PinMode.kModeDigitalOut, PinMode.kModeDigitalOut, PinMode.kModeDigitalOut]

        self.write_queue = Queue()
        self.i2c_lock = Lock()

        self.left_encoder = Encoder(self._read_left_encoder())
        self.right_encoder = Encoder(self._read_right_encoder())

    
    def set_io_config(self, pin11=None, pin4=None, pin20=None, pin21=None, pin22=None):
        pins = [pin11, pin4, pin20, pin21, pin22]
        pin_names = [11, 4, 20, 21, 22]

        config_register = 1 << 15  # 15th bit signals the controller to set the pin config
        for index, pin_mode in enumerate(pins):
            if pin_mode is None:
                pin_mode = self.io_modes[index]
            elif not PinMode.is_mode(pin_mode):
                warnings.warn("Pin %s config is not a valid mode: %s" % (pin_names[index], pin_mode))
                pin_mode = self.io_modes[index]
            else:
                self.io_modes[index] = pin_mode
            
            config_register |= (pin_mode & 0x3) << (13 - (2 * index))
        
        self.write("ioConfig", config_register)
    
    def cap_velocity(self, velocity, reverse=False):
        # velocity: -1.0...1.0
        if velocity > 1.0:
            velocity = 1.0
        if velocity < -1.0:
            velocity = -1.0
        if reverse:
            velocity *= -1
        
        velocity = int(0x7fff * velocity)
        return velocity

    def set_left_motor(self, velocity):
        velocity = self.cap_velocity(velocity)
        self.write("leftMotor", velocity)
    
    def set_right_motor(self, velocity):
        velocity = self.cap_velocity(velocity, True)
        self.write("rightMotor", velocity)
    
    def reset_left_encoder(self):
        self.write("resetLeftEncoder", 1)
    
    def reset_right_encoder(self):
        self.write("resetRightEncoder", 1)
    
    def get_left_encoder(self):
        self.left_encoder.update(self._read_left_encoder())
        return self.left_encoder.ticks * self.ticks_to_m
    
    def get_right_encoder(self):
        self.right_encoder.update(self._read_right_encoder())
        return self.right_encoder.ticks * self.ticks_to_m
    
    def get_ultrasonic_dist_1(self):
        return self._read_ultrasonic("ultrasonicDist1")
    
    def get_ultrasonic_dist_2(self):
        return self._read_ultrasonic("ultrasonicDist2")
    
    def _read_ultrasonic(self, key):
        dist = self.read(key)
        if dist == HCSR04_OUT_OF_RANGE:
            return float("nan")
        else:
            return dist
    
    def _read_left_encoder(self):
        return self.read("leftEncoder")
    
    def _read_right_encoder(self):
        return self.read("rightEncoder")
    
    def heartbeat(self):
        self.write("heartbeat", 1)

    def write(self, register_name, data):
        offset = self.sharedmem_map[register_name]["offset"]
        mem_format = self.sharedmem_map[register_name]["format"]
        data_array = [struct.unpack('b', x)[0] for x in struct.pack(mem_format, data)]
        self.write_queue.put((offset, data_array))

    def read(self, register_name):
        offset = self.sharedmem_map[register_name]["offset"]
        mem_format = self.sharedmem_map[register_name]["format"]
        size = self.sharedmem_map[register_name]["size"]

        with self.i2c_lock:
            self.bus.write_byte(self.address, offset)
            time.sleep(0.001)
            byte_list = [self.bus.read_byte(self.address) for _ in range(size)]
        data = struct.unpack(mem_format, bytearray(byte_list))
        if len(data) == 1:
            return data[0]
        else:
            return data
        
    def update(self):
        with self.i2c_lock:
            while not self.write_queue.empty():
                offset, data_array = self.write_queue.get()
                self._write(offset, data_array)

    def _write(self, offset, data_array):
        self.bus.write_i2c_block_data(self.address, offset, data_array)
        time.sleep(0.001)

    def load_config(self, sharedmem_path):
        with open(sharedmem_path) as file:
            contents = json.load(file)
        
        mem_map = {}
        offset = 0
        for mem_register in contents:
            mem_format, size = self.type_to_format(mem_register.get("type"), mem_register.get("arraySize", 1))
            mem_map[mem_register["name"]] = {
                "offset": offset,
                "format": mem_format,
                "size": size,
            }
            offset += size
        return mem_map
    
    def type_to_format(self, mem_type, array_size):
        if mem_type == "uint16_t":
            mem_format = "H"
            size = 2
        elif mem_type == "int16_t":
            mem_format = "h"
            size = 2
        elif mem_type == "uint8_t":
            mem_format = "B"
            size = 1
        elif mem_type == "int8_t":
            mem_format = "b"
            size = 1
        elif mem_type == "bool":
            mem_format = "b"
            size = 1
        elif mem_type == "float":
            mem_format = "f"
            size = 4
        else:
            raise ValueError("Invalid mem_type: %s" % mem_type)
        
        mem_format = mem_format * array_size
        size *= array_size
        return mem_format, size
