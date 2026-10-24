#!/usr/bin/env python3
"""Frontier-based autonomous exploration.

Detects boundaries between known free space and unknown space in the
occupancy grid, clusters them, and navigates to the best frontier
until the entire environment is mapped.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy import ndimage
import math
import tf2_ros


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('exploration_timeout', 30.0)
        self.declare_parameter('blacklist_radius', 0.5)
        self.declare_parameter('recheck_interval', 1.0)

        self._min_frontier_size = (
            self.get_parameter('min_frontier_size')
            .get_parameter_value().integer_value
        )
        self._exploration_timeout = (
            self.get_parameter('exploration_timeout')
            .get_parameter_value().double_value
        )
        self._blacklist_radius = (
            self.get_parameter('blacklist_radius')
            .get_parameter_value().double_value
        )
        recheck_interval = (
            self.get_parameter('recheck_interval')
            .get_parameter_value().double_value
        )

        # State
        self.current_map = None
        self.is_navigating = False
        self.current_target = None
        self.blacklisted_frontiers = []
        self._visited_frontiers = []
        self.consecutive_failures = 0
        self._total_goals_sent = 0
        self._total_goals_reached = 0
        self._goal_handle = None
        self._goal_start_time = None
        self._timed_out = False
        self._preempted = False
        self._map_count = 0
        self._wait_map_count = None
        self._wait_map_deadline = None
        self._frontier_mask = None
        self._rep_lookup = {}
        self._current_rep = None

        # TF buffer for robot pose
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribe to SLAM map
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Frontier visualization
        self._frontier_pub = self.create_publisher(
            MarkerArray, 'frontier_markers', 10
        )

        # Exploration loop timer
        self.create_timer(recheck_interval, self.explore_step)
        self.create_timer(1.0, self._check_goal_timeout)

        self.get_logger().info('Frontier explorer initialized')

    def map_callback(self, msg: OccupancyGrid):
        self.current_map = msg
        self._map_count += 1

    def _get_robot_pose(self):
        """Get robot position in map frame via TF."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            return (
                t.transform.translation.x,
                t.transform.translation.y,
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def explore_step(self):
        """One iteration of the exploration loop."""
        if self.current_map is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=10.0)
            return

        if self.is_navigating:
            self._preempt_if_explored()
            return

        # After reaching a goal, wait for the next map update so it
        # includes what the robot saw there; otherwise a stale map makes the
        # frontier look unobservable. Capped, because SLAM Toolbox doesn't
        # publish anything new while the robot stands still.
        if self._wait_map_count is not None:
            if (self._map_count <= self._wait_map_count
                    and self.get_clock().now() < self._wait_map_deadline):
                return
            self._wait_map_count = None

        if self.consecutive_failures > 5:
            self.get_logger().warn(
                'Too many consecutive failures — clearing blacklist'
            )
            self.blacklisted_frontiers.clear()
            self.consecutive_failures = 0

        frontiers = self._detect_frontiers()
        self._publish_frontier_markers(frontiers)

        if len(frontiers) == 0:
            self._log_completion()
            return

        robot_pose = self._get_robot_pose()
        target = self._select_frontier(frontiers, robot_pose)

        if target is None:
            # Every remaining frontier is right under the robot and still
            # hasn't cleared: it can't be observed from here. Without this
            # nothing is ever selectable again and the explorer idles.
            for c in frontiers:
                self.get_logger().warn(
                    f"Frontier ({c['x']:.2f}, {c['y']:.2f}) is at the robot "
                    f'but not clearing — blacklisting as unobservable'
                )
                self.blacklisted_frontiers.append((c['x'], c['y']))
            return

        # Reaching a frontier that still doesn't clear after a second visit
        # means its unknown cells can't be observed (e.g. shadowed by an
        # obstacle). Without this the explorer re-selects it forever.
        visits = sum(
            math.hypot(target[0] - vx, target[1] - vy) < self._blacklist_radius
            for vx, vy in self._visited_frontiers
        )
        if visits >= 2:
            self.get_logger().warn(
                f'Frontier ({target[0]:.2f}, {target[1]:.2f}) persists '
                f'after being reached — blacklisting as unobservable'
            )
            self.blacklisted_frontiers.append(target)
            return

        self._navigate_to(target, robot_pose)
        self._current_rep = self._rep_lookup.get(target, target)

    def _preempt_if_explored(self):
        """Cancel the active goal once its frontier has been mapped.

        The laser sees well ahead of the robot, so a frontier often clears
        while the robot is still driving to it.
        """
        if (self._goal_handle is None or self._preempted
                or self.current_target is None):
            return
        self._detect_frontiers()
        if self._frontier_near(self._current_rep or self.current_target):
            return
        self.get_logger().info(
            f'Frontier ({self.current_target[0]:.2f}, '
            f'{self.current_target[1]:.2f}) already explored — moving on'
        )
        self._preempted = True
        self._goal_handle.cancel_goal_async()

    def _frontier_near(self, target):
        """True if any frontier cell lies within blacklist_radius of target."""
        if self._frontier_mask is None:
            return True
        info = self.current_map.info
        col = int((target[0] - info.origin.position.x) / info.resolution)
        row = int((target[1] - info.origin.position.y) / info.resolution)
        r = max(1, int(self._blacklist_radius / info.resolution))
        window = self._frontier_mask[
            max(0, row - r):row + r + 1, max(0, col - r):col + r + 1
        ]
        return bool(window.any())

    def _detect_frontiers(self):
        """Find and cluster frontier cells in the occupancy grid."""
        grid = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width,
        )

        free_mask = (grid == 0)
        unknown_mask = (grid == -1)

        # Find free cells that are adjacent to unknown cells
        kernel = np.ones((3, 3), dtype=bool)
        free_dilated = ndimage.binary_dilation(free_mask, structure=kernel)
        frontier_mask = free_dilated & unknown_mask

        frontier_coords = np.argwhere(frontier_mask)
        if len(frontier_coords) == 0:
            self._frontier_mask = np.zeros_like(frontier_mask)
            return []

        return self._cluster_frontiers(frontier_coords)

    def _cluster_frontiers(self, coords):
        """Group frontier cells into connected clusters."""
        grid = np.zeros(
            (self.current_map.info.height, self.current_map.info.width),
            dtype=bool,
        )
        grid[coords[:, 0], coords[:, 1]] = True
        labeled, n_clusters = ndimage.label(grid)
        # Only clusters big enough to count; single-cell SLAM speckles
        # would otherwise make every spot look unexplored.
        sizes = ndimage.sum(grid, labeled, range(1, n_clusters + 1))
        keep = np.flatnonzero(sizes >= self._min_frontier_size) + 1
        self._frontier_mask = np.isin(labeled, keep)

        info = self.current_map.info
        clusters = []
        self._rep_lookup = {}

        for i in range(1, n_clusters + 1):
            cluster_cells = np.argwhere(labeled == i)
            if len(cluster_cells) < self._min_frontier_size:
                continue

            centroid = cluster_cells.mean(axis=0)
            # Convert grid coords (row, col) to world coords (x, y)
            wx = info.origin.position.x + centroid[1] * info.resolution
            wy = info.origin.position.y + centroid[0] * info.resolution
            # The centroid of a curved frontier can sit away from every
            # frontier cell, so remember a real one for preemption checks.
            # (The centroid stays the goal: it often lies through a gap,
            # which is what gets the robot into a narrow entrance.)
            nearest = cluster_cells[
                np.argmin(((cluster_cells - centroid) ** 2).sum(axis=1))
            ]
            self._rep_lookup[(wx, wy)] = (
                info.origin.position.x + (nearest[1] + 0.5) * info.resolution,
                info.origin.position.y + (nearest[0] + 0.5) * info.resolution,
            )

            # Skip blacklisted locations
            if self._is_blacklisted(wx, wy):
                continue

            clusters.append({
                'x': wx,
                'y': wy,
                'size': len(cluster_cells),
            })

        return clusters

    def _is_blacklisted(self, x, y):
        """Check if a frontier is near a previously blacklisted location."""
        for bx, by in self.blacklisted_frontiers:
            dist = math.sqrt((x - bx) ** 2 + (y - by) ** 2)
            if dist < self._blacklist_radius:
                return True
        return False

    def _select_frontier(self, clusters, robot_pose):
        """Pick the best frontier balancing size and proximity."""
        if not clusters:
            return None

        best = None
        best_score = -float('inf')

        for c in clusters:
            # Score: balance between cluster size and distance to robot
            size_score = c['size']

            if robot_pose is not None:
                dist = math.sqrt(
                    (c['x'] - robot_pose[0]) ** 2
                    + (c['y'] - robot_pose[1]) ** 2
                )
                # Avoid very close frontiers (likely noise)
                if dist < 0.3:
                    continue
                # Prefer closer, larger frontiers
                score = size_score / (dist + 1.0)
            else:
                score = size_score

            if score > best_score:
                best_score = score
                best = (c['x'], c['y'])

        return best

    def _navigate_to(self, target, robot_pose=None):
        """Send a navigation goal to Nav2."""
        self.is_navigating = True
        self.current_target = target
        self._current_rep = None
        self._preempted = False
        self._total_goals_sent += 1
        gx, gy = target

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        # Face along the direction of travel so the robot doesn't spin on
        # the spot at the goal to match an arbitrary heading.
        yaw = 0.0
        if robot_pose is not None:
            yaw = math.atan2(gy - robot_pose[1], gx - robot_pose[0])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'[{self._total_goals_sent}] Navigating to frontier '
            f'({target[0]:.2f}, {target[1]:.2f})'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            self.is_navigating = False
            return

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.is_navigating = False
            self.consecutive_failures += 1
            return

        self._goal_handle = goal_handle
        self._goal_start_time = self.get_clock().now()
        self._timed_out = False
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_complete_cb)

    def _check_goal_timeout(self):
        """Cancel the active goal if it exceeds exploration_timeout."""
        if self._goal_handle is None or self._goal_start_time is None:
            return
        elapsed = (
            self.get_clock().now() - self._goal_start_time
        ).nanoseconds / 1e9
        if elapsed > self._exploration_timeout:
            self.get_logger().warn(
                f'Goal exceeded {self._exploration_timeout:.0f}s timeout '
                f'— canceling and blacklisting'
            )
            self._timed_out = True
            self._goal_start_time = None
            self._goal_handle.cancel_goal_async()

    def _nav_complete_cb(self, future):
        result = future.result()
        status = result.status
        self._goal_handle = None
        self._goal_start_time = None

        if self._preempted and not self._timed_out:
            # We canceled it ourselves because the frontier got mapped
            self._preempted = False
            self.consecutive_failures = 0
        elif status == GoalStatus.STATUS_ABORTED or self._timed_out:
            self.get_logger().warn(
                f'Goal aborted — blacklisting '
                f'({self.current_target[0]:.2f}, {self.current_target[1]:.2f})'
            )
            self.blacklisted_frontiers.append(self.current_target)
            self.consecutive_failures += 1
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled')
            self.consecutive_failures += 1
        else:
            self.get_logger().info('Reached frontier, scanning for next...')
            self._total_goals_reached += 1
            self._wait_map_count = self._map_count
            self._wait_map_deadline = \
                self.get_clock().now() + Duration(seconds=3.0)
            self._visited_frontiers.append(self.current_target)
            self.consecutive_failures = 0

        self.is_navigating = False

    def _log_completion(self):
        """Log exploration statistics."""
        grid = np.array(self.current_map.data)
        free = np.sum(grid == 0)
        unknown = np.sum(grid == -1)
        occupied = np.sum(grid > 0)
        total_known = free + occupied

        completeness = (
            total_known / (total_known + unknown) * 100
            if (total_known + unknown) > 0
            else 0
        )

        self.get_logger().info(
            f'No frontiers remaining — exploration complete!\n'
            f'  Map completeness: {completeness:.1f}%\n'
            f'  Goals sent: {self._total_goals_sent}\n'
            f'  Goals reached: {self._total_goals_reached}\n'
            f'  Blacklisted: {len(self.blacklisted_frontiers)}'
        )

    def _publish_frontier_markers(self, clusters):
        """Publish frontier cluster centroids as RViz markers."""
        marker_array = MarkerArray()

        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, c in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = c['x']
            marker.pose.position.y = c['y']
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.2

            # Color by size: larger clusters are greener
            marker.color.r = 0.2
            marker.color.g = min(1.0, c['size'] / 50.0)
            marker.color.b = 0.8
            marker.color.a = 0.7
            marker_array.markers.append(marker)

        self._frontier_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
