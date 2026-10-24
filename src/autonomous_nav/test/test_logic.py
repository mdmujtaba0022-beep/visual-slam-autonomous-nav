"""Unit tests for frontier detection, waypoint loading, utils and map metrics."""
import math

import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid

from autonomous_nav import utils
from autonomous_nav.frontier_explorer import FrontierExplorer
from autonomous_nav.metrics import compare_maps
from autonomous_nav.waypoint_manager import WaypointManager


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_grid(data, resolution=0.1, ox=0.0, oy=0.0):
    msg = OccupancyGrid()
    msg.info.height, msg.info.width = data.shape
    msg.info.resolution = resolution
    msg.info.origin.position.x = ox
    msg.info.origin.position.y = oy
    msg.data = data.flatten().astype(int).tolist()
    return msg


def test_yaw_quaternion_roundtrip():
    for yaw in (-3.0, -1.57, 0.0, 0.5, 3.0):
        assert utils.quaternion_to_yaw(utils.yaw_to_quaternion(yaw)) == \
            pytest.approx(yaw)


def test_make_pose_and_distance():
    p = utils.make_pose(1.0, 2.0, math.pi / 2)
    assert p.header.frame_id == 'map'
    assert (p.pose.position.x, p.pose.position.y) == (1.0, 2.0)
    assert utils.euclidean_distance(0, 0, 3, 4) == pytest.approx(5.0)


def test_frontier_detection_finds_boundary():
    node = FrontierExplorer()
    try:
        grid = np.full((20, 20), -1)
        grid[:, :10] = 0  # left half known free, right half unknown
        node.current_map = make_grid(grid)
        frontiers = node._detect_frontiers()
        assert len(frontiers) == 1
        f = frontiers[0]
        assert f['size'] == 20
        # Frontier column is 10 -> x = 1.0 m at 0.1 m/cell
        assert f['x'] == pytest.approx(1.0)
        assert f['y'] == pytest.approx(0.95)
    finally:
        node.destroy_node()


def test_fully_known_map_has_no_frontiers():
    node = FrontierExplorer()
    try:
        grid = np.zeros((10, 10))
        grid[0, :] = 100
        node.current_map = make_grid(grid)
        assert node._detect_frontiers() == []
    finally:
        node.destroy_node()


def test_blacklist_and_selection():
    node = FrontierExplorer()
    try:
        node.blacklisted_frontiers = [(1.0, 1.0)]
        assert node._is_blacklisted(1.2, 1.0)
        assert not node._is_blacklisted(2.0, 1.0)
        clusters = [
            {'x': 0.1, 'y': 0.0, 'size': 100},   # too close -> skipped
            {'x': 2.0, 'y': 0.0, 'size': 30},
            {'x': 8.0, 'y': 0.0, 'size': 40},
        ]
        assert node._select_frontier(clusters, (0.0, 0.0)) == (2.0, 0.0)
        assert node._select_frontier(clusters, None) == (0.1, 0.0)
    finally:
        node.destroy_node()


def test_reached_frontier_that_persists_gets_blacklisted():
    node = FrontierExplorer()
    try:
        grid = np.full((20, 20), -1)
        grid[:, :10] = 0
        node.current_map = make_grid(grid)
        sent = []
        node._navigate_to = sent.append
        node.explore_step()
        assert sent == [(pytest.approx(1.0), pytest.approx(0.95))]
        # Nav2 reports success but the map doesn't change
        node.current_target = sent[0]
        node._nav_complete_cb(_Future(GoalStatus.STATUS_SUCCEEDED))
        node.explore_step()
        assert len(sent) == 1
        assert len(node.blacklisted_frontiers) == 1
    finally:
        node.destroy_node()


class _Result:
    def __init__(self, status):
        self.status = status


class _Future:
    def __init__(self, status):
        self._r = _Result(status)

    def result(self):
        return self._r


@pytest.mark.parametrize('status,blacklisted,reached', [
    (GoalStatus.STATUS_SUCCEEDED, 0, 1),
    (GoalStatus.STATUS_ABORTED, 1, 0),
    (GoalStatus.STATUS_CANCELED, 0, 0),
])
def test_nav_result_status_handling(status, blacklisted, reached):
    node = FrontierExplorer()
    try:
        node.current_target = (1.0, 2.0)
        node.is_navigating = True
        node._nav_complete_cb(_Future(status))
        assert len(node.blacklisted_frontiers) == blacklisted
        assert node._total_goals_reached == reached
        assert not node.is_navigating
    finally:
        node.destroy_node()


def test_waypoint_markers_do_not_mutate_goals(tmp_path):
    wp_file = tmp_path / 'mission.yaml'
    wp_file.write_text(
        'waypoints:\n'
        '  - {x: 1.0, y: -2.0, yaw: 0.0}\n'
        '  - {x: 3.0, y: 4.0, yaw: 1.57}\n'
    )
    node = WaypointManager.__new__(WaypointManager)
    rclpy.node.Node.__init__(node, 'waypoint_manager_test')
    try:
        node._marker_pub = node.create_publisher(
            __import__('visualization_msgs.msg', fromlist=['MarkerArray'])
            .MarkerArray, 'waypoint_markers', 10)
        node.waypoints = node._load_waypoints(str(wp_file))
        assert len(node.waypoints) == 2
        assert node.waypoints[1].pose.orientation.z == \
            pytest.approx(math.sin(1.57 / 2))
        node._publish_markers()
        assert all(w.pose.position.z == 0.0 for w in node.waypoints)
    finally:
        node.destroy_node()


def test_compare_maps(tmp_path):
    from PIL import Image
    a = np.full((10, 10), 254, dtype=np.uint8)
    a[0, :] = 0
    b = a.copy()
    b[5, 5] = 205  # one unknown cell
    Image.fromarray(a).save(tmp_path / 'gt.pgm')
    Image.fromarray(b).save(tmp_path / 'gen.pgm')
    r = compare_maps(str(tmp_path / 'gen.pgm'), str(tmp_path / 'gt.pgm'))
    assert r['occupied_iou'] == pytest.approx(1.0)
    assert r['free_precision'] == pytest.approx(1.0)
    assert r['free_recall'] == pytest.approx(89 / 90)
