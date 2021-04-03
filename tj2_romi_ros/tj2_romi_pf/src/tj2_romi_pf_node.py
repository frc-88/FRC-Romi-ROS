#!/usr/bin/env python

import rospy

import numpy as np

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

from nav_msgs.srv import GetMap

from std_msgs.msg import Float64

import tf2_ros

from particle_filter import ParticleFilter
from map_raytracer import MapRaytracer


class TJ2RomiPfNode:
    def __init__(self):
        self.node_name = "tj2_romi_pf"
        rospy.init_node(
            self.node_name,
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown_hook)

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_service_name = rospy.get_param("~static_map", "static_map")

        self.show_particles = rospy.get_param("~publish_particles", True)

        self.initial_range = rospy.get_param("~initial_range", None)
        self.input_std = rospy.get_param("~input_std", None)
        self.meas_std_val = rospy.get_param("~meas_std_val", 0.007)
        self.num_particles = rospy.get_param("~num_particles", 100)
        self.input_std = rospy.get_param("~input_std", None)

        if self.initial_range is None:
            self.initial_range = [1.0, 1.0, 1.0]
        if self.input_std is None:
            self.input_std = [0.007, 0.007]
        self.input_vector = np.zeros(len(self.input_std))

        self.pf = ParticleFilter(self.num_particles, self.meas_std_val, self.input_std)
        self.prev_pf_time = rospy.Time.now().to_sec()

        self.map_raytracer = MapRaytracer(self.base_frame, "ultrasonic1", "ultrasonic2")

        self.ultrasonic1_sub = rospy.Subscriber("ultrasonic1", Float64, self.ultrasonic1_callback, queue_size=25)
        self.ultrasonic2_sub = rospy.Subscriber("ultrasonic2", Float64, self.ultrasonic2_callback, queue_size=25)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=25)

        self.particles_pub = rospy.Publisher("pf_particles", PoseArray, queue_size=5)

        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.get_map = rospy.ServiceProxy(self.map_service_name, GetMap)
        rospy.loginfo("Waiting for service %s" % self.map_service_name)
        rospy.wait_for_service(self.map_service_name)
        self.map_msg = self.get_map().map
        self.map_raytracer.set_map(self.map_msg)

        rospy.loginfo("%s init done" % self.node_name)
        
    def ultrasonic1_callback(self, msg):
        self.pf.update(msg.data, "ultrasonic1")
    
    def ultrasonic2_callback(self, msg):
        self.pf.update(msg.data, "ultrasonic2")

    def odom_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        dt = current_time - self.prev_pf_time
        self.prev_pf_time = current_time

        self.input_vector[0] = -msg.twist.twist.linear.x
        self.input_vector[1] = -msg.twist.twist.angular.z
        self.pf.predict(self.input_vector, dt)

    def publish_all_pose(self):
        mean = self.pf.mean()

        # TODO: first cancel out odom transform, then apply map transform
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.transform.translation.x = mean[0]
        msg.transform.translation.y = mean[1]
        msg.transform.translation.z = mean[2]
        msg.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(msg)
    
    def publish_particles(self):
        particles_msg = PoseArray()
        particles_msg.header.frame_id = self.map_frame
        particles_msg.header.stamp = rospy.Time.now()

        for particle in  self.pf.particles:
            pose_msg = Pose()
            pose_msg.position.x = particle[0]
            pose_msg.position.y = particle[1]
            pose_msg.position.z = particle[2]
            particles_msg.poses.append(pose_msg)

        self.particles_pub.publish(particles_msg)

    def run(self):
        rate = rospy.Rate(30.0)

        while True:
            rate.sleep()
            if rospy.is_shutdown():
                break
            self.pf.check_resample()

            self.publish_all_pose()
            
            if self.show_particles:
                self.publish_particles()
        # rospy.spin()


if __name__ == "__main__":
    node = TJ2RomiPfNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
