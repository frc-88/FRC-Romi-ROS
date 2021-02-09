#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from networktables import NetworkTables


class TJ2RomiNetworkTables(object):
    def __init__(self):
        self.node_name = "tj2_romi_networktables"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        # self.host_ip = rospy.get_param("~host_ip", "0.0.0.0")
        # NetworkTables.initialize(server=self.host_ip)
        NetworkTables.startServer("networktables.ini", "0.0.0.0")
        self.nt = NetworkTables.getTable("ROS")

        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=500)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu"

        self.enc_left_pub = rospy.Publisher("encoder_left", Float64, queue_size=500)
        self.enc_left_msg = Float64()

        self.enc_right_pub = rospy.Publisher("encoder_right", Float64, queue_size=500)
        self.enc_right_msg = Float64()

        self.inch_to_m = 0.0254

        self.remote_start_time = None
        self.local_start_time = None
        
        self.clock_rate = rospy.Rate(30.0)


    def run(self):
        self.wait_for_time()

        while not rospy.is_shutdown():
            self.publish_imu()
            self.publish_enc()

            self.clock_rate.sleep()
    
    def wait_for_time(self):
        while self.remote_start_time is None:
            self.remote_start_time = self.nt.getEntry("accel/time").getDouble(None)
            self.clock_rate.sleep()
            if rospy.is_shutdown():
                return
        self.local_start_time = rospy.Time.now().to_sec()

    def publish_imu(self):
        self.imu_msg.header.stamp = rospy.Time.from_sec(self.get_romi_time())
        self.imu_msg.linear_acceleration.x = self.get_table("accel/x")
        self.imu_msg.linear_acceleration.y = self.get_table("accel/y")
        self.imu_msg.linear_acceleration.z = self.get_table("accel/z")
        self.imu_msg.angular_velocity.x = self.get_table("gyro/x")
        self.imu_msg.angular_velocity.y = self.get_table("gyro/y")
        self.imu_msg.angular_velocity.z = self.get_table("gyro/z")
        
        self.imu_pub.publish(self.imu_msg)
    
    def publish_enc(self):
        left_dist = self.get_table("encoder/left") * self.inch_to_m
        right_dist = self.get_table("encoder/right") * self.inch_to_m

        if left_dist != self.enc_left_msg.data:
            self.enc_left_msg.data = left_dist
            self.enc_left_pub.publish(self.enc_left_msg)
        if right_dist != self.enc_right_msg.data:
            self.enc_right_msg.data = right_dist
            self.enc_right_pub.publish(self.enc_right_msg)

    def get_romi_time(self):
        timestamp = self.nt.getEntry("accel/time").getDouble(0.0)
        return timestamp - self.remote_start_time + self.local_start_time
        

    def get_table(self, path):
        return self.nt.getEntry(path).getDouble(0.0)

if __name__ == "__main__":
    node = TJ2RomiNetworkTables()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
