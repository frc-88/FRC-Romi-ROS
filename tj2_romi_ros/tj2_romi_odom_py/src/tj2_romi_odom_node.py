#!/usr/bin/python
import rospy


class TJ2RomiOdom:
    def __init__(self):
        self.node_name = "tj2_romi_odom_py"
        rospy.init_node(self.node_name)

    def run(self):
        rospy.spin()



if __name__ == "__main__":
    node = TJ2RomiOdom()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
