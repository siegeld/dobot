#!/usr/bin/env python3
"""
ROS2 Action Server for synchronous joint moves.

Wraps the MovJ service and provides feedback until motion completes.
"""

import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dobot_actions.action import MoveJoints
from dobot_msgs_v4.srv import MovJ, GetAngle


class MoveJointsServer(Node):
    """Action server for synchronous joint moves."""

    def __init__(self):
        super().__init__('move_joints_server')

        self._cb_group = ReentrantCallbackGroup()

        # Service clients
        self._movj_client = self.create_client(
            MovJ,
            '/dobot_bringup_ros2/srv/MovJ',
            callback_group=self._cb_group
        )
        self._get_angle_client = self.create_client(
            GetAngle,
            '/dobot_bringup_ros2/srv/GetAngle',
            callback_group=self._cb_group
        )

        # Action server
        self._action_server = ActionServer(
            self,
            MoveJoints,
            'move_joints',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

        self.get_logger().info('MoveJoints action server ready')

    def goal_callback(self, goal_request):
        """Accept or reject goal."""
        self.get_logger().info(f'Received goal: {goal_request.target}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def get_joint_angles(self):
        """Get current joint angles."""
        request = GetAngle.Request()
        future = self._get_angle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            return None

        # Parse response: "{j1,j2,j3,j4,j5,j6}"
        import re
        match = re.search(r'\{([^}]+)\}', future.result().robot_return)
        if match:
            values = match.group(1).split(',')
            return [float(v.strip()) for v in values]
        return None

    async def execute_callback(self, goal_handle):
        """Execute the move and provide feedback."""
        self.get_logger().info('Executing move...')

        target = list(goal_handle.request.target)
        tolerance = goal_handle.request.tolerance or 0.5

        # Send MovJ command
        movj_request = MovJ.Request()
        movj_request.mode = True
        movj_request.a = target[0]
        movj_request.b = target[1]
        movj_request.c = target[2]
        movj_request.d = target[3]
        movj_request.e = target[4]
        movj_request.f = target[5]
        movj_request.param_value = []

        future = self._movj_client.call_async(movj_request)
        await future

        if future.result() is None:
            goal_handle.abort()
            result = MoveJoints.Result()
            result.success = False
            result.message = 'MovJ service call failed'
            return result

        # Poll until reached or cancelled
        feedback = MoveJoints.Feedback()
        timeout = 30.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveJoints.Result()
                result.success = False
                result.message = 'Cancelled'
                return result

            current = self.get_joint_angles()
            if current is None:
                await self._sleep(0.1)
                continue

            max_error = max(abs(current[i] - target[i]) for i in range(6))

            # Publish feedback
            feedback.current_position = current
            feedback.max_error = max_error
            goal_handle.publish_feedback(feedback)

            if max_error <= tolerance:
                goal_handle.succeed()
                result = MoveJoints.Result()
                result.success = True
                result.message = 'Target reached'
                result.final_position = current
                return result

            await self._sleep(0.1)

        # Timeout
        goal_handle.abort()
        result = MoveJoints.Result()
        result.success = False
        result.message = 'Timeout'
        result.final_position = self.get_joint_angles() or [0.0] * 6
        return result

    async def _sleep(self, duration):
        """Async sleep."""
        import asyncio
        await asyncio.sleep(duration)


def main(args=None):
    rclpy.init(args=args)
    node = MoveJointsServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
