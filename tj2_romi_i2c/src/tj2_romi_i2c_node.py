#!/usr/bin/python
import rospy
from smbus import SMBus

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

from romi_i2c import RomiI2C
from imu_i2c import ImuI2C


class TJ2RomiI2C(object):
    def __init__(self):
        self.node_name = "tj2_romi_i2c"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        self.sharedmem_path = rospy.get_param("~sharedmem_path", "./sharedmem.json")

        self.accel_offset_x = rospy.get_param("~accel_offset_x", 1.0)
        self.accel_offset_y = rospy.get_param("~accel_offset_y", 1.0)
        self.accel_offset_z = rospy.get_param("~accel_offset_z", 1.0)

        self.gyro_offset_x = rospy.get_param("~gyro_offset_x", 0.0)
        self.gyro_offset_y = rospy.get_param("~gyro_offset_y", 0.0)
        self.gyro_offset_z = rospy.get_param("~gyro_offset_z", 0.0)

        self.bus = SMBus(1)
        self.romi_i2c = RomiI2C(self.bus, self.sharedmem_path)
        self.imu_i2c = ImuI2C(
            self.bus,
            accel_scale="SCALE_2G",
            gyro_scale="SCALE_250_DPS",
            accel_offsets=[self.accel_offset_x, self.accel_offset_y, self.accel_offset_z],
            gyro_offsets=[self.gyro_offset_x, self.gyro_offset_y, self.gyro_offset_z],
        )

        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=50)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu"

        self.imu_msg.angular_velocity_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 0.25,
        ]
        
        self.imu_msg.linear_acceleration_covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 0.25,
        ]

        self.enc_left_pub = rospy.Publisher("encoder_left", Float64, queue_size=50)
        self.enc_left_msg = Float64()

        self.enc_right_pub = rospy.Publisher("encoder_right", Float64, queue_size=50)
        self.enc_right_msg = Float64()

        self.motor_left_sub = rospy.Subscriber("motor_left", Float64, self.motor_left_callback, queue_size=50)
        self.motor_right_sub = rospy.Subscriber("motor_right", Float64, self.motor_right_callback, queue_size=50)

        self.remote_start_time = None
        self.local_start_time = None
        self.prev_timestamp = 0.0

        self.heartbeat_timer = rospy.Timer(rospy.Duration(0.25), self.heartbeat_callback)
        
        self.clock_rate = rospy.Rate(30.0)

    def reinit_bus(self):
        self.bus.close()
        del self.bus
        rospy.sleep(0.25)
        self.bus = SMBus(1)

    def run(self):
        # offset applied in RomiI2C
        # self.romi_i2c.reset_left_encoder()
        # self.romi_i2c.reset_right_encoder()

        while not rospy.is_shutdown():
            try:
                self.romi_i2c.update()
                
                self.publish_imu()
                self.publish_enc()
            except IOError as e:
                self.reinit_bus()
                rospy.logwarn("Reinitialized I2C bus: %s" % (str(e)))

            self.clock_rate.sleep()
    
    def publish_imu(self):
        self.imu_i2c.update()

        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.linear_acceleration.x = self.imu_i2c.accel.x
        self.imu_msg.linear_acceleration.y = self.imu_i2c.accel.y
        self.imu_msg.linear_acceleration.z = self.imu_i2c.accel.z
        self.imu_msg.angular_velocity.x = self.imu_i2c.gyro.x
        self.imu_msg.angular_velocity.y = self.imu_i2c.gyro.y
        self.imu_msg.angular_velocity.z = self.imu_i2c.gyro.z
        
        self.imu_pub.publish(self.imu_msg)
    
    def publish_enc(self):
        left_dist = self.romi_i2c.get_left_encoder()
        right_dist = self.romi_i2c.get_right_encoder()

        if left_dist != self.enc_left_msg.data:
            self.enc_left_msg.data = left_dist
            self.enc_left_pub.publish(self.enc_left_msg)
        if right_dist != self.enc_right_msg.data:
            self.enc_right_msg.data = right_dist
            self.enc_right_pub.publish(self.enc_right_msg)

    def motor_left_callback(self, msg):
        self.romi_i2c.set_left_motor(msg.data)
    
    def motor_right_callback(self, msg):
        self.romi_i2c.set_right_motor(msg.data)
    
    def heartbeat_callback(self, timer):
        self.romi_i2c.heartbeat()


if __name__ == "__main__":
    node = TJ2RomiI2C()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
