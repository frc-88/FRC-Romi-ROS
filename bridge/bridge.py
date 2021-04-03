import time
import warnings
from smbus import SMBus

from romi_bus import RomiI2C
from romi_bus import ImuI2C

from romi_server import RomiServer


class Bridge:
    def __init__(self, **kwargs):
        self.sharedmem_path = kwargs.get("sharedmem_path", "./sharedmem.json")
        self.accel_offset_x = kwargs.get("~accel_offset_x", 1.0)
        self.accel_offset_y = kwargs.get("~accel_offset_y", 1.0)
        self.accel_offset_z = kwargs.get("~accel_offset_z", 1.0)

        self.gyro_offset_x = kwargs.get("~gyro_offset_x", 0.0)
        self.gyro_offset_y = kwargs.get("~gyro_offset_y", 0.0)
        self.gyro_offset_z = kwargs.get("~gyro_offset_z", 0.0)

        self.wheel_diameter = kwargs.get("~wheel_diameter_m", 0.0545)

        update_rate = kwargs.get("~update_rate", 30.0)
        self.update_delay = 1.0 / update_rate

        self.bus = SMBus(1)
        self.romi_i2c = RomiI2C(self.bus, self.sharedmem_path, self.wheel_diameter)
        self.imu_i2c = ImuI2C(
            self.bus,
            accel_scale="SCALE_2G",
            gyro_scale="SCALE_250_DPS",
            accel_offsets=[self.accel_offset_x, self.accel_offset_y, self.accel_offset_z],
            gyro_offsets=[self.gyro_offset_x, self.gyro_offset_y, self.gyro_offset_z],
        )

        self.server = RomiServer()

        self.accel_msg = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }
        self.gyro_msg = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
        }

        self.encoder_msg = {
            "left": 0.0,
            "right": 0.0,
        }

        self.ultrasonic_msg = {
            "1": 0.0,
            "2": 0.0,
        }

        self.heartbeat_timer = time.time()
        self.heartbeat_interval = 0.25

    def reinit_bus(self):
        self.bus.close()
        del self.bus
        time.sleep(0.25)
        self.bus = SMBus(1)

    def run(self):
        while True:
            try:
                self.romi_i2c.update()
                self.imu_i2c.update()
                
                self.publish_imu()
                self.publish_enc()
                self.publish_ultrasonic()

                self.check_heartbeat()

                self.set_motors()
            except IOError as e:
                self.reinit_bus()
                warnings.warn("Reinitialized I2C bus: %s" % (str(e)))

            time.sleep(self.update_delay)
    
    def publish_imu(self):
        self.accel_msg["x"] = self.imu_i2c.accel.x
        self.accel_msg["y"] = self.imu_i2c.accel.y
        self.accel_msg["z"] = self.imu_i2c.accel.z

        self.gyro_msg["x"] = self.imu_i2c.gyro.x
        self.gyro_msg["y"] = self.imu_i2c.gyro.y
        self.gyro_msg["z"] = self.imu_i2c.gyro.z
        self.server.push("imu/accel", self.accel_msg)
        self.server.push("imu/gyro", self.gyro_msg)

    def publish_enc(self):
        self.encoder_msg["left"] = self.romi_i2c.get_left_encoder()
        self.encoder_msg["right"] = self.romi_i2c.get_right_encoder()
        self.server.push("encoders", self.encoder_msg)

    def publish_ultrasonic(self):
        self.ultrasonic_msg["1"] = self.romi_i2c.get_ultrasonic_dist_1()
        self.ultrasonic_msg["2"] = self.romi_i2c.get_ultrasonic_dist_2()
        self.server.push("ultrasonic", self.ultrasonic_msg)

    def check_heartbeat(self):
        if time.time() - self.heartbeat_timer < self.heartbeat_interval:
            return
        self.romi_i2c.heartbeat()
    
    def set_motors(self):
        self.romi_i2c.set_left_motor(self.server.pull("motors/left", 0.0))
        self.romi_i2c.set_right_motor(self.server.pull("motors/right", 0.0))

