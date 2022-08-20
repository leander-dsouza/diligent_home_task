#! /usr/bin/env python3
"""Program to publish regular polygonal paths."""
import argparse
import tf2_ros
import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from diligent_home_task.compute_waypoints import compute_waypoints

bash_colours = {
    'green': '\033[1;32m',
    'reset': '\033[0m'
}


class PolygonalPath():
    """Polygonal Path Class."""

    def __init__(self, num_sides, length):
        """Initialize helper variables."""
        self.num_sides = num_sides
        self.length = length

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path_publisher = \
            rospy.Publisher('/robot_markers', MarkerArray,
                            latch=True, queue_size=10)

        self.control_loop()

    def go_to_pose(self, goal_pose):
        """
        Move the robot to a given goal pose.

        Args:
            goal_pose (list): The goal pose to move to.
        Returns:
            True if the goal is reached, False otherwise.

        """
        client = SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_pose[0]
        goal.target_pose.pose.position.y = goal_pose[1]

        orientation_list = quaternion_from_euler(0.0, 0.0, goal_pose[2])
        goal.target_pose.pose.orientation.x = orientation_list[0]
        goal.target_pose.pose.orientation.y = orientation_list[1]
        goal.target_pose.pose.orientation.z = orientation_list[2]
        goal.target_pose.pose.orientation.w = orientation_list[3]

        client.send_goal(goal)

        if not client.wait_for_result():
            rospy.signal_shutdown("Action server not available!")
            return False
        return client.get_result()

    def pose_handler(self, pose_list):
        """
        Handle the poses recieved, and shutdown node when achieved.

        Args:
            pose_list (list): The list of poses to move to.
        """
        for [x_coord, y_coord, yaw] in pose_list:
            result = self.go_to_pose([x_coord, y_coord, yaw])
            if result:
                rospy.loginfo(
                    bash_colours['green'] + "Reached waypoint" +
                    bash_colours['reset'])
        rospy.signal_shutdown("Node Shutdown")

    def plot_polygon(self, pose_list):
        """
        Plot the polygon in RViz using markers to describe edges and waypoints.

        Args:
            pose_list (list): The list of poses to move to.
        """
        marker_array = MarkerArray()
        marker_id = 0

        edges = Marker()
        edges.header.frame_id = "map"
        edges.header.stamp = rospy.Time.now()
        edges.type = Marker.LINE_STRIP
        edges.action = Marker.ADD
        edges.id = marker_id

        edges.scale.x = 0.1
        edges.scale.y = 0.1
        edges.scale.z = 0.1

        edges.color.r = 1.0
        edges.color.g = 0.0
        edges.color.b = 0.0
        edges.color.a = 1.0

        for [x_coord, y_coord, _] in pose_list:
            pnt = Point()
            pnt.x = x_coord
            pnt.y = y_coord
            edges.points.append(pnt)

            vertex = Marker()
            vertex.header.frame_id = "map"
            vertex.header.stamp = rospy.Time.now()
            vertex.type = vertex.SPHERE
            vertex.action = vertex.ADD
            marker_id += 1
            vertex.id = marker_id

            vertex.scale.x = 0.4
            vertex.scale.y = 0.4
            vertex.scale.z = 0.4

            vertex.color.r = 1.0
            vertex.color.g = 1.0
            vertex.color.b = 0.0
            vertex.color.a = 1.0

            vertex.pose.position.x = x_coord
            vertex.pose.position.y = y_coord
            marker_array.markers.append(vertex)

        edges.points.append(edges.points[0])
        marker_array.markers.append(edges)
        self.path_publisher.publish(marker_array)

    def convert_tf_to_pose2d(self, transform):
        """
        Transform the robot's position from tf to pose2d.

        Convert the transform recieved to a 2D pose and
        invoke calls to plot_polygon and pose_handler.

        Args:
            transform (geometry_msgs.msg.TransformStamped):
                The transform to convert to.
        """
        pose_x = transform.transform.translation.x
        pose_y = transform.transform.translation.y

        pose_theta = euler_from_quaternion([
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w])[2]

        waypoints = compute_waypoints(
            self.num_sides, self.length,
            origin=[pose_x, pose_y, pose_theta])

        self.plot_polygon(waypoints)
        self.pose_handler(waypoints)

    def get_origin(self):
        """
        Get the robot's origin from tf.

        Get the current position of the robot and
        invoke convert_tf_to_pose2d to proceed.

        Returns:
            True/False (bool): If the robot's position
                was successfully retrieved.

        """
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time())
            self.convert_tf_to_pose2d(transform)
            return True

        except (
            tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn("TF Exception")
            return False

    def control_loop(self):
        """Invoke the robot's trigger steps."""
        try:
            while not rospy.is_shutdown():
                if self.get_origin():
                    compute_waypoints(self.num_sides, self.length)
                rospy.sleep(0.1)

        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("Done")


def main():
    """Start the node processes."""
    parser = argparse.ArgumentParser(
        description='Publish Regular Polygonal Paths')
    parser.add_argument(
        '-n', '--num_sides', type=int,
        default=6, help='Number of sides of the polygon')
    parser.add_argument(
        '-l', '--length', type=float,
        default=10.0, help='Length of the polygon')
    args = parser.parse_args()

    rospy.init_node('polygonal_path_node')
    PolygonalPath(args.num_sides, args.length)
    rospy.spin()


if __name__ == '__main__':
    main()
