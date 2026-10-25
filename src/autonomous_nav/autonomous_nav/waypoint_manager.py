#!/usr/bin/env python3
"""Multi-waypoint mission executor using Nav2 FollowWaypoints action."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import copy
import yaml
import math


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self._waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints'
        )

        self.declare_parameter('waypoints_file', '')
        waypoints_file = (
            self.get_parameter('waypoints_file')
            .get_parameter_value()
            .string_value
        )

        if not waypoints_file:
            self.get_logger().error('No waypoints_file parameter provided')
            return

        self.waypoints = self._load_waypoints(waypoints_file)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')

        # Publish waypoint markers for RViz visualization
        self._marker_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10
        )
        self.create_timer(1.0, self._publish_markers)

        # Start mission after a brief delay for Nav2 to initialize
        self.create_timer(3.0, self._start_mission_once)
        self._mission_started = False
        self._retries_left = 20
        self._retry_timer = None

    def _load_waypoints(self, filepath):
        """Load waypoints from a YAML file."""
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        waypoints = []
        for wp in data['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            pose.pose.position.z = 0.0
            yaw = float(wp.get('yaw', 0.0))
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            waypoints.append(pose)

        return waypoints

    def _start_mission_once(self):
        if self._mission_started:
            return
        self._mission_started = True
        self.execute_mission()

    def execute_mission(self):
        """Send all waypoints as a FollowWaypoints goal."""
        self.get_logger().info('Waiting for follow_waypoints action server...')
        if not self._waypoint_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints

        self.get_logger().info(
            f'Sending mission with {len(self.waypoints)} waypoints...'
        )
        future = self._waypoint_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)

    def _retry_mission(self):
        self._retry_timer.cancel()
        self.destroy_timer(self._retry_timer)
        self._retry_timer = None
        self.execute_mission()

    def _feedback_cb(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(
            f'Navigating to waypoint {current + 1}/{len(self.waypoints)}'
        )

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # The action server exists before the lifecycle manager
            # activates it, and an inactive server rejects goals.
            if self._retries_left > 0:
                self._retries_left -= 1
                self.get_logger().warn(
                    'Mission rejected (Nav2 not active yet?) — retrying in 5s'
                )
                self._retry_timer = self.create_timer(5.0, self._retry_mission)
                return
            self.get_logger().error('Mission rejected by Nav2!')
            return

        self.get_logger().info('Mission accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        missed = result.missed_waypoints
        total = len(self.waypoints)
        reached = total - len(missed)

        if len(missed) == 0:
            self.get_logger().info(
                f'Mission complete! All {total} waypoints reached.'
            )
        else:
            self.get_logger().warn(
                f'Mission complete. Reached {reached}/{total}. '
                f'Missed indices: {list(missed)}'
            )

    def _publish_markers(self):
        """Publish waypoint markers for RViz visualization."""
        marker_array = MarkerArray()

        for i, wp in enumerate(self.waypoints):
            # Sphere marker at waypoint position
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(wp.pose)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            # Copy so the label offset doesn't mutate the mission goal
            text_marker.pose = copy.deepcopy(wp.pose)
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'WP{i}'
            marker_array.markers.append(text_marker)

        self._marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
