"""Real-robot adapters implementing the Robot / World protocols on top of
DobotRosClient and the camera server. Every motion passes the shared
guards and (optionally) the pick_confirm broker before it is issued."""
from dobot_ros.real.robot import RealRobot  # noqa: F401
from dobot_ros.real.world import RealWorld  # noqa: F401
