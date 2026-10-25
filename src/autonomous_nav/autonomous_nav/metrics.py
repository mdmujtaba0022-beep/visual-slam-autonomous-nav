#!/usr/bin/env python3
"""Exploration and SLAM metrics collection node.

Subscribes to map, odometry, and Gazebo ground truth to compute:
- Map completeness over time
- Path efficiency (distance traveled vs optimal)
- Localization error (SLAM pose vs ground truth)
- Navigation success rate
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import tf2_ros
import math
import csv
import os


class ExplorationMetrics(Node):
    def __init__(self):
        super().__init__('exploration_metrics')

        self.declare_parameter('output_dir', 'metrics_output')
        self.declare_parameter('sample_interval', 2.0)

        self._output_dir = (
            self.get_parameter('output_dir')
            .get_parameter_value().string_value
        )
        sample_interval = (
            self.get_parameter('sample_interval')
            .get_parameter_value().double_value
        )

        os.makedirs(self._output_dir, exist_ok=True)

        # State
        self._start_time = None
        self._current_map = None
        self._distance_traveled = 0.0
        self._last_odom_pose = None
        self._goals_sent = 0
        self._goals_reached = 0
        self._completeness_log = []

        # TF for ground truth comparison
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribers
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, 10
        )
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10
        )

        # Periodic metrics sampling
        self.create_timer(sample_interval, self._sample_metrics)

        self.get_logger().info('Metrics collection started')

    def _map_cb(self, msg: OccupancyGrid):
        self._current_map = msg
        if self._start_time is None:
            self._start_time = self.get_clock().now()

    def _odom_cb(self, msg: Odometry):
        """Track total distance traveled."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_odom_pose is not None:
            dx = x - self._last_odom_pose[0]
            dy = y - self._last_odom_pose[1]
            self._distance_traveled += math.sqrt(dx * dx + dy * dy)

        self._last_odom_pose = (x, y)

    def _sample_metrics(self):
        """Record current metrics snapshot."""
        if self._current_map is None or self._start_time is None:
            return

        elapsed = (
            (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        )
        completeness = self._compute_completeness()

        # Get localization error if ground truth available
        loc_error = self._compute_localization_error()

        entry = {
            'time_s': round(elapsed, 1),
            'completeness_pct': round(completeness, 2),
            'distance_m': round(self._distance_traveled, 2),
            'localization_error_m': round(loc_error, 4) if loc_error else None,
        }
        self._completeness_log.append(entry)

        self.get_logger().info(
            f'[{elapsed:.0f}s] completeness={completeness:.1f}% '
            f'dist={self._distance_traveled:.1f}m',
            throttle_duration_sec=10.0,
        )

    def _compute_completeness(self):
        """Compute percentage of map that is known (free + occupied)."""
        import numpy as np
        grid = np.array(self._current_map.data)
        free = int(np.sum(grid == 0))
        unknown = int(np.sum(grid == -1))
        occupied = int(np.sum(grid > 0))
        total_known = free + occupied
        total = total_known + unknown
        return (total_known / total * 100) if total > 0 else 0.0

    def _compute_localization_error(self):
        """Compare SLAM pose against Gazebo ground truth if available."""
        try:
            # SLAM estimate: map -> base_link
            slam_tf = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            sx = slam_tf.transform.translation.x
            sy = slam_tf.transform.translation.y

            # Ground truth from Gazebo (if published)
            gt_tf = self._tf_buffer.lookup_transform(
                'map', 'base_link_gt', rclpy.time.Time()
            )
            gx = gt_tf.transform.translation.x
            gy = gt_tf.transform.translation.y

            return math.sqrt((sx - gx) ** 2 + (sy - gy) ** 2)
        except Exception:
            return None

    def save_results(self):
        """Write collected metrics to CSV."""
        csv_path = os.path.join(self._output_dir, 'exploration_metrics.csv')
        if not self._completeness_log:
            return

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(
                f, fieldnames=self._completeness_log[0].keys()
            )
            writer.writeheader()
            writer.writerows(self._completeness_log)

        self.get_logger().info(f'Metrics saved to {csv_path}')

    def get_summary(self):
        """Return final exploration summary."""
        if not self._completeness_log:
            return {}

        final = self._completeness_log[-1]
        return {
            'total_time_s': final['time_s'],
            'final_completeness_pct': final['completeness_pct'],
            'total_distance_m': final['distance_m'],
            'path_efficiency': (
                final['distance_m'] / max(final['completeness_pct'], 1)
            ),
        }


def compare_maps(generated_pgm: str, ground_truth_pgm: str) -> dict:
    """Compare a generated map against ground truth.

    Args:
        generated_pgm: Path to SLAM-generated .pgm map file.
        ground_truth_pgm: Path to ground truth .pgm map file.

    Returns:
        Dict with IoU, precision, recall metrics.
    """
    from PIL import Image
    import numpy as np

    gen = np.array(Image.open(generated_pgm))
    gt = np.array(Image.open(ground_truth_pgm))

    # Resize if dimensions differ
    if gen.shape != gt.shape:
        gt_img = Image.open(ground_truth_pgm).resize(
            (gen.shape[1], gen.shape[0]), Image.NEAREST
        )
        gt = np.array(gt_img)

    # Binarize: free space = white = 1. map_saver writes unknown as 205,
    # so the threshold must sit above it or unknown counts as free.
    gen_free = (gen > 205).astype(int)
    gt_free = (gt > 205).astype(int)

    intersection = int(np.sum(gen_free & gt_free))
    union = int(np.sum(gen_free | gt_free))

    # Occupied space comparison
    gen_occ = (gen < 50).astype(int)
    gt_occ = (gt < 50).astype(int)
    occ_intersection = int(np.sum(gen_occ & gt_occ))
    occ_union = int(np.sum(gen_occ | gt_occ))

    return {
        'free_space_iou': intersection / union if union > 0 else 0,
        'occupied_iou': occ_intersection / occ_union if occ_union > 0 else 0,
        'free_precision': (
            intersection / int(np.sum(gen_free))
            if np.sum(gen_free) > 0 else 0
        ),
        'free_recall': (
            intersection / int(np.sum(gt_free))
            if np.sum(gt_free) > 0 else 0
        ),
    }


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        summary = node.get_summary()
        if summary:
            node.get_logger().info(f'Final summary: {summary}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
