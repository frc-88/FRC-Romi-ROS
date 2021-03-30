import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf_conversions.transformations import quaternion_from_euler, euler_from_quaternion
import range_libc


class MapRaytracer(object):
    def __init__(self, base_frame, *distance_sensor_serials, max_range_meters=5.0, theta_discretization=112):
        self.serials = distance_sensor_serials
        self.map = None
        self.range_method = None

        self.base_frame = base_frame

        self.max_range_px = 0
        self.max_range_meters = max_range_meters
        self.theta_discretization = theta_discretization

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = self.base_frame

        self.base_to_sensor_tfs = {}
        for serial in self.serials:
            self.base_to_sensor_tfs[serial] = self._lookup_transform(serial, self.base_frame)

    def set_map(self, occupancy_grid):
        self.map = range_libc.PyOMap(occupancy_grid)
        self.max_range_px = int(self.max_range_meters / occupancy_grid.info.resolution)
        self.range_method = range_libc.PyCDDTCast(self.map, self.max_range_px, self.theta_discretization)

    def trace(self, state, serial):
        base_to_sensor = self.base_to_sensor_tfs[serial]
        self.pose_stamped.pose.position.x = state[0]
        self.pose_stamped.pose.position.y = state[1]
        self.pose_stamped.pose.orientation = quaternion_from_euler(0.0, 0.0, state[2])
        
        pose_sensor_frame = tf2_geometry_msgs.do_transform_pose(self.pose_stamped, base_to_sensor)

        # ray trace
        trace_angle = euler_from_quaternion([
            pose_sensor_frame.pose.orientation.w,
            pose_sensor_frame.pose.orientation.x,
            pose_sensor_frame.pose.orientation.y,
            pose_sensor_frame.pose.orientation.z,
        ])[2]
        distance = self.range_method.calc_range(
            pose_sensor_frame.pose.position.x,
            pose_sensor_frame.pose.position.y,
            trace_angle
        )

        return distance

    def _lookup_transform(self, parent_link, child_link, time_window=None, timeout=None, raise_exception=True):
        """
        Call tf_buffer.lookup_transform. Return None if the look up fails
        """
        if time_window is None:
            time_window = rospy.Time(0)
        else:
            time_window = rospy.Time.now() - time_window

        if timeout is None:
            timeout = rospy.Duration(1.0)

        try:
            return self.tf_buffer.lookup_transform(parent_link, child_link, time_window, timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (parent_link, child_link, e))
            if raise_exception:
                raise
