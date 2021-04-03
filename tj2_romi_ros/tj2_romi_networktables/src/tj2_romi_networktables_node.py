#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

from romi_i2c import RomiI2C
from imu_i2c import ImuI2C

# To see messages from networktables, you must setup logging
import logging

from networktables import NetworkTables


logging.basicConfig(level=logging.DEBUG)


class TJ2RomiI2C(object):
    def __init__(self):
        self.node_name = "tj2_romi_i2c"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        
        self.host_ip = rospy.get_param("~host_ip", "0.0.0.0")
        NetworkTables.initialize(server=self.host_ip)
        self.table = NetworkTables.getTable("romi")
        
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

        self.ultrasonic1_pub = rospy.Publisher("ultrasonic1", Float64, queue_size=50)
        self.ultrasonic1_msg = Float64()

        self.ultrasonic2_pub = rospy.Publisher("ultrasonic2", Float64, queue_size=50)
        self.ultrasonic2_msg = Float64()

        self.motor_left_sub = rospy.Subscriber("motor_left", Float64, self.motor_left_callback, queue_size=50)
        self.motor_right_sub = rospy.Subscriber("motor_right", Float64, self.motor_right_callback, queue_size=50)

        self.remote_start_time = None
        self.local_start_time = None
        self.prev_timestamp = 0.0

        self.ultrasonic_timer = rospy.Timer(rospy.Duration(1.0 / 10.0), self.ultrasonic_callback)

        self.clock_rate = rospy.Rate(30.0)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_imu()
            self.publish_enc()
            self.clock_rate.sleep()
    
    def publish_imu(self):
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.linear_acceleration.x = self.pull_value("imu/accel/x")
        self.imu_msg.linear_acceleration.y = self.pull_value("imu/accel/y")
        self.imu_msg.linear_acceleration.z = self.pull_value("imu/accel/z")
        self.imu_msg.angular_velocity.x = self.pull_value("imu/gyro/x")
        self.imu_msg.angular_velocity.y = self.pull_value("imu/gyro/y")
        self.imu_msg.angular_velocity.z = self.pull_value("imu/gyro/z")
        
        self.imu_pub.publish(self.imu_msg)
    
    def publish_enc(self):
        left_dist = self.pull_value("encoders/left")
        right_dist = self.pull_value("encoders/right")

        if left_dist != self.enc_left_msg.data:
            self.enc_left_msg.data = left_dist
            self.enc_left_pub.publish(self.enc_left_msg)
        if right_dist != self.enc_right_msg.data:
            self.enc_right_msg.data = right_dist
            self.enc_right_pub.publish(self.enc_right_msg)
    
    def publish_ultrasonic(self):
        self.ultrasonic1_msg.data = self.pull_value("ultrasonic/1")
        self.ultrasonic2_msg.data = self.pull_value("ultrasonic/2")
        self.ultrasonic1_pub.publish(self.ultrasonic1_msg)
        self.ultrasonic2_pub.publish(self.ultrasonic2_msg)

    def ultrasonic_callback(self, timer):
        self.publish_ultrasonic()
    
    def motor_left_callback(self, msg):
        self.push_value("motors/left", msg.data)
    
    def motor_right_callback(self, msg):
        self.push_value("motors/right", msg.data)
    
    def pull_value(self, root, default=0.0):
        return self.table.getNumber(root, default)

    def push_value(self, path, value):
        self.table.putNumber(path, value)

if __name__ == "__main__":
    node = TJ2RomiI2C()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
