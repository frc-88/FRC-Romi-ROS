import math
import struct


class Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
    
    @classmethod
    def from_list(cls, l):
        assert len(l) == 3, "len(%s) != 3" % len(l)
        self = cls()
        self.x = l[0]
        self.y = l[1]
        self.z = l[2]
        return self


class AccelScale:
    SCALE_2G = 0.061
    SCALE_4G = 0.122
    SCALE_8G = 0.244
    SCALE_16G = 0.488

    SCALE_2G_CTRL = 0x40
    SCALE_4G_CTRL = 0x48
    SCALE_8G_CTRL = 0x4C
    SCALE_16G_CTRL = 0x44

    @classmethod
    def get(cls, name):
        return cls.__dict__.get(name, 0.0)

class GyroScale:
    SCALE_125_DPS = 4.375
    SCALE_250_DPS = 8.75
    SCALE_500_DPS = 17.5
    SCALE_1000_DPS = 35
    SCALE_2000_DPS = 70

    SCALE_125_DPS_CTRL = 0x42
    SCALE_250_DPS_CTRL = 0x40
    SCALE_500_DPS_CTRL = 0x44
    SCALE_1000_DPS_CTRL = 0x48
    SCALE_2000_DPS_CTRL = 0x4C

    @classmethod
    def get(cls, name):
        return cls.__dict__.get(name, 0.0)
        
# Register Addresses
FUNC_CFG_ACCESS   = 0x01

FIFO_CTRL1        = 0x06
FIFO_CTRL2        = 0x07
FIFO_CTRL3        = 0x08
FIFO_CTRL4        = 0x09
FIFO_CTRL5        = 0x0A
ORIENT_CFG_G      = 0x0B

INT1_CTRL         = 0x0D
INT2_CTRL         = 0x0E
WHO_AM_I          = 0x0F
CTRL1_XL          = 0x10
CTRL2_G           = 0x11
CTRL3_C           = 0x12
CTRL4_C           = 0x13
CTRL5_C           = 0x14
CTRL6_C           = 0x15
CTRL7_G           = 0x16
CTRL8_XL          = 0x17
CTRL9_XL          = 0x18
CTRL10_C          = 0x19

WAKE_UP_SRC       = 0x1B
TAP_SRC           = 0x1C
D6D_SRC           = 0x1D
STATUS_REG        = 0x1E

OUT_TEMP_L        = 0x20
OUT_TEMP_H        = 0x21
OUTX_L_G          = 0x22
OUTX_H_G          = 0x23
OUTY_L_G          = 0x24
OUTY_H_G          = 0x25
OUTZ_L_G          = 0x26
OUTZ_H_G          = 0x27
OUTX_L_XL         = 0x28
OUTX_H_XL         = 0x29
OUTY_L_XL         = 0x2A
OUTY_H_XL         = 0x2B
OUTZ_L_XL         = 0x2C
OUTZ_H_XL         = 0x2D

FIFO_STATUS1      = 0x3A
FIFO_STATUS2      = 0x3B
FIFO_STATUS3      = 0x3C
FIFO_STATUS4      = 0x3D
FIFO_DATA_OUT_L   = 0x3E
FIFO_DATA_OUT_H   = 0x3F
TIMESTAMP0_REG    = 0x40
TIMESTAMP1_REG    = 0x41
TIMESTAMP2_REG    = 0x42

STEP_TIMESTAMP_L  = 0x49
STEP_TIMESTAMP_H  = 0x4A
STEP_COUNTER_L    = 0x4B
STEP_COUNTER_H    = 0x4C

FUNC_SRC          = 0x53

TAP_CFG           = 0x58
TAP_THS_6D        = 0x59
INT_DUR2          = 0x5A
WAKE_UP_THS       = 0x5B
WAKE_UP_DUR       = 0x5C
FREE_FALL         = 0x5D
MD1_CFG           = 0x5E
MD2_CFG           = 0x5F

# IMU constants
DS33_WHO_ID = 0x69
IF_INC_ENABLED = 0x04
ACCEL_G = 9.81


class ImuI2C(object):
    def __init__(self, bus, accel_scale="SCALE_2G", gyro_scale="SCALE_250_DPS", accel_offsets=None, gyro_offsets=None):
        self.bus = bus
        self.address = 0x6B

        self.accel = Vector3()
        self.gyro = Vector3()

        self.accel_scale = accel_scale
        self.gyro_scale = gyro_scale
        self.accel_scale_val = AccelScale.get(self.accel_scale)
        self.gyro_scale_val = GyroScale.get(self.gyro_scale)

        if accel_offsets is None:
            self.accel_offsets = Vector3()
        else:
            self.accel_offsets = Vector3.from_list(accel_offsets)
        
        if gyro_offsets is None:
            self.gyro_offsets = Vector3()
        else:
            self.gyro_offsets = Vector3.from_list(gyro_offsets)

        self._who_am_i()

        self.set_scales(self.accel_scale, self.gyro_scale)
    
    def update(self):
        self.accel.x = self._read_accel_x()  # m/s^2
        self.accel.y = self._read_accel_y()  # m/s^2
        self.accel.z = self._read_accel_z()  # m/s^2
        self.gyro.x = math.radians(self._read_gyro_x())  # rad/s
        self.gyro.y = math.radians(self._read_gyro_y())  # rad/s
        self.gyro.z = math.radians(self._read_gyro_z())  # rad/s

    def set_scales(self, accel_scale, gyro_scale):
        self._set_accel_scale(accel_scale)
        self._set_gyro_scale(gyro_scale)
        self.bus.write_i2c_block_data(self.address, CTRL3_C, [IF_INC_ENABLED])

    def _set_accel_scale(self, scale):
        self.accel_scale = scale
        ctrl_byte = AccelScale.get(scale + "_CTRL")
        self.accel_scale_val = AccelScale.get(scale)
        
        self.bus.write_i2c_block_data(self.address, CTRL1_XL, [ctrl_byte])

    def _set_gyro_scale(self, scale):
        self.accel_scale = scale
        ctrl_byte = GyroScale.get(scale + "_CTRL")
        self.gyro_scale_val = GyroScale.get(scale)

        self.bus.write_i2c_block_data(self.address, CTRL2_G, [ctrl_byte])

    def _who_am_i(self):
        data = self._read_byte(WHO_AM_I)
        if data != DS33_WHO_ID:
            raise Exception("Invalid WHO_AM_I response from IMU: %s != %s" % (data, DS33_WHO_ID))
    
    def _read_imu(self, high_reg, low_reg):
        byte_list = [0, 0]
        byte_list[0] = self._read_byte(low_reg)
        byte_list[1] = self._read_byte(high_reg)
        data = struct.unpack('h', bytearray(byte_list))
        return data[0]

    def _read_accel_x(self):
        data = self._read_imu(OUTX_H_XL, OUTX_L_XL)
        return (data * self.accel_scale_val) / 1000.0 * ACCEL_G * self.accel_offsets.x

    def _read_accel_y(self):
        data = self._read_imu(OUTY_H_XL, OUTY_L_XL)
        return (data * self.accel_scale_val) / 1000.0 * ACCEL_G * self.accel_offsets.y

    def _read_accel_z(self):
        data = self._read_imu(OUTZ_H_XL, OUTZ_L_XL)
        return (data * self.accel_scale_val) / 1000.0 * ACCEL_G * self.accel_offsets.z

    def _read_gyro_x(self):
        data = self._read_imu(OUTX_H_G, OUTX_L_G)
        return (data * self.gyro_scale_val) / 1000.0 - self.gyro_offsets.x

    def _read_gyro_y(self):
        data = self._read_imu(OUTY_H_G, OUTY_L_G)
        return (data * self.gyro_scale_val) / 1000.0 - self.gyro_offsets.y

    def _read_gyro_z(self):
        data = self._read_imu(OUTZ_H_G, OUTZ_L_G)
        return (data * self.gyro_scale_val) / 1000.0 - self.gyro_offsets.z

    def _read_byte(self, register):
        self.bus.write_byte(self.address, register)
        time.sleep(0.001)
        return self.bus.read_byte(self.address)
