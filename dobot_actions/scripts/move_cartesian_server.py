#!/usr/bin/env python3
"""
ROS2 Action Server for synchronous cartesian moves.

Wraps RelMovLUser/RelMovLTool services and provides feedback until motion completes.
"""

import math
import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dobot_actions.action import MoveCartesian
from dobot_msgs_v4.srv import RelMovLUser, RelMovLTool, GetPose


class MoveCartesianServer(Node):
    """Action server for synchronous cartesian moves."""

    def __init__(self):
        super().__init__('move_cartesian_server')

        self._cb_group = ReentrantCallbackGroup()

        # Service clients
        self._rel_mov_user_client = self.create_client(
            RelMovLUser,
            '/dobot_bringup_ros2/srv/RelMovLUser',
            callback_group=self._cb_group
        )
        self._rel_mov_tool_client = self.create_client(
            RelMovLTool,
            '/dobot_bringup_ros2/srv/RelMovLTool',
            callback_group=self._cb_group
        )
        self._get_pose_client = self.create_client(
            GetPose,
            '/dobot_bringup_ros2/srv/GetPose',
            callback_group=self._cb_group
        )

        # Action server
        self._action_server = ActionServer(
            self,
            MoveCartesian,
            'move_cartesian',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group
        )

        self.get_logger().info('MoveCartesian action server ready')

    def goal_callback(self, goal_request):
        """Accept or reject goal."""
        self.get_logger().info(
            f'Received goal: x={goal_request.x}, y={goal_request.y}, z={goal_request.z}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def get_cartesian_pose(self):
        """Get current cartesian pose [X, Y, Z, RX, RY, RZ]."""
        import re
        request = GetPose.Request()
        request.user = 0
        request.tool = 0
        future = self._get_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            return None

        # Parse response: "{x,y,z,rx,ry,rz}"
        match = re.search(r'\{([^}]+)\}', future.result().robot_return)
        if match:
            values = match.group(1).split(',')
            return [float(v.strip()) for v in values]
        return None

    async def execute_callback(self, goal_handle):
        """Execute the move and provide feedback."""
        self.get_logger().info('Executing cartesian move...')

        # Get goal parameters
        dx = goal_handle.request.x
        dy = goal_handle.request.y
        dz = goal_handle.request.z
        drx = goal_handle.request.rx
        dry = goal_handle.request.ry
        drz = goal_handle.request.rz
        mode = goal_handle.request.mode or 'user'
        tolerance = goal_handle.request.tolerance or 1.0

        # Get starting position to calculate target
        start_pose = self.get_cartesian_pose()
        if start_pose is None:
            goal_handle.abort()
            result = MoveCartesian.Result()
            result.success = False
            result.message = 'Failed to get current pose'
            return result

        # Calculate target (relative move)
        target = [
            start_pose[0] + dx,
            start_pose[1] + dy,
            start_pose[2] + dz,
            start_pose[3] + drx,
            start_pose[4] + dry,
            start_pose[5] + drz,
        ]

        # Send RelMovL command
        if mode.lower() == 'tool':
            request = RelMovLTool.Request()
            client = self._rel_mov_tool_client
        else:
            request = RelMovLUser.Request()
            client = self._rel_mov_user_client

        request.a = dx
        request.b = dy
        request.c = dz
        request.d = drx
        request.e = dry
        request.f = drz
        request.param_value = []

        future = client.call_async(request)
        await future

        if future.result() is None:
            goal_handle.abort()
            result = MoveCartesian.Result()
            result.success = False
            result.message = 'RelMovL service call failed'
            return result

        # Poll until reached or cancelled
        feedback = MoveCartesian.Feedback()
        timeout = 30.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveCartesian.Result()
                result.success = False
                result.message = 'Cancelled'
                return result

            current = self.get_cartesian_pose()
            if current is None:
                await self._sleep(0.1)
                continue

            # Calculate distance remaining (position only, not rotation)
            dist = math.sqrt(
                (current[0] - target[0])**2 +
                (current[1] - target[1])**2 +
                (current[2] - target[2])**2
            )

            # Publish feedback
            feedback.current_position = current
            feedback.distance_remaining = dist
            goal_handle.publish_feedback(feedback)

            if dist <= tolerance:
                goal_handle.succeed()
                result = MoveCartesian.Result()
                result.success = True
                result.message = 'Target reached'
                result.final_position = current
                return result

            await self._sleep(0.1)

        # Timeout
        goal_handle.abort()
        result = MoveCartesian.Result()
        result.success = False
        result.message = 'Timeout'
        result.final_position = self.get_cartesian_pose() or [0.0] * 6
        return result

    async def _sleep(self, duration):
        """Async sleep."""
        import asyncio
        await asyncio.sleep(duration)


def main(args=None):
    rclpy.init(args=args)
    node = MoveCartesianServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
