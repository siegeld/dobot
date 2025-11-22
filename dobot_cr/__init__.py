"""
Dobot CR Controller - Professional CLI for Dobot CR series collaborative robots.
"""

__version__ = "0.0.1"
__author__ = "Dobot CR Controller Contributors"
__license__ = "MIT"

from dobot_cr.config import Config
from dobot_cr.robot import DobotController, Position

__all__ = ["Config", "DobotController", "Position", "__version__"]
