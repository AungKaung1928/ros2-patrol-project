"""Launch test: the installed patrol_controller against a stand-in Nav2 action server.

The test process serves navigate_to_pose: goal 1 is aborted, later goals succeed.
With the shipped config (retry_on_failure: true) the node must send the same patrol
point again, then move on to the next point after reaching it.
Run: launch_test test/test_launch_retry.py (after sourcing the workspace).
"""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='patrol_navigation_project', executable='patrol_controller', output='screen')
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest(), node]), {'node': node}


class TestRetry(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_aborted_point_is_retried_then_patrol_moves_on(self, proc_output, node):
        fake = rclpy.create_node('fake_nav2')
        goals = []

        def execute(goal_handle):
            p = goal_handle.request.pose.pose.position
            goals.append((round(p.x, 3), round(p.y, 3)))
            if len(goals) == 1:
                goal_handle.abort()
            else:
                goal_handle.succeed()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute)
        executor = MultiThreadedExecutor()
        executor.add_node(fake)
        try:
            start = time.monotonic()  # the node waits ~17 s for Nav2 before its first goal
            while len(goals) < 3 and time.monotonic() - start < 90.0:
                executor.spin_once(timeout_sec=0.1)
        finally:
            server.destroy()
            executor.shutdown()
            fake.destroy_node()
        self.assertGreaterEqual(len(goals), 3, f'goals seen: {goals}')
        # config/patrol_points.yaml: origin_station (0, 0), then point_2 (2, 2)
        self.assertEqual(goals[:3], [(0.0, 0.0), (0.0, 0.0), (2.0, 2.0)])
        proc_output.assertWaitFor('Retry 1/3', process=node, timeout=5)
