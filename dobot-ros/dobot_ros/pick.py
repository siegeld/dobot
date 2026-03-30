"""Pick sequence — camera-guided robot pick-and-place.

Uses VisionClient to locate objects, then commands the robot via
DobotRosClient to pick them up.
"""

import logging
import time
from dataclasses import dataclass
from typing import Optional

from .vision import VisionClient, DetectedObject, RobotPoint

logger = logging.getLogger(__name__)


@dataclass
class PickConfig:
    """Configuration for pick operations."""
    approach_height_mm: float = 100.0    # Z offset above object for approach
    grasp_height_mm: float = 5.0         # Z offset for actual grasp (above table)
    lift_height_mm: float = 150.0        # Z to lift after grasping
    gripper_open_pos: int = 800          # gripper position for open
    gripper_close_pos: int = 200         # gripper position for close
    gripper_force: int = 50              # grip force (20-100)
    gripper_speed: int = 50              # grip speed (1-100)
    move_speed: int = 30                 # robot speed factor (1-100)
    place_xyz: Optional[list] = None     # [X, Y, Z] robot coords to place object


class PickExecutor:
    """Executes camera-guided pick sequences."""

    def __init__(self, robot_client, vision: VisionClient, config: PickConfig = None):
        """
        Args:
            robot_client: DobotRosClient instance (already connected)
            vision: VisionClient instance
            config: Pick configuration
        """
        self.robot = robot_client
        self.vision = vision
        self.config = config or PickConfig()

    def find_object(self, object_id: int = None) -> Optional[DetectedObject]:
        """Find target object. If no ID given, picks the largest object."""
        objects = self.vision.get_objects()
        if not objects:
            logger.warning("No objects detected")
            return None

        if object_id is not None:
            for obj in objects:
                if obj.id == object_id:
                    return obj
            logger.warning(f"Object #{object_id} not found")
            return None

        # Pick largest by area
        return max(objects, key=lambda o: o.area_px)

    def get_robot_target(self, obj: DetectedObject) -> RobotPoint:
        """Transform object position to robot coordinates."""
        return self.vision.transform_to_robot(obj.position_3d)

    def pick(self, object_id: int = None, log_callback=None) -> bool:
        """Execute a full pick sequence.

        Args:
            object_id: Specific object ID to pick, or None for largest
            log_callback: Optional function(message) for status updates

        Returns:
            True if pick succeeded
        """
        def log(msg):
            logger.info(msg)
            if log_callback:
                log_callback(msg)

        # 1. Find target object
        log("Scanning for target object...")
        obj = self.find_object(object_id)
        if obj is None:
            log("No target object found")
            return False

        log(f"Target: #{obj.id} at camera ({obj.x:.3f}, {obj.y:.3f}, {obj.z:.3f})m, "
            f"rotation={obj.rotation_deg:.0f}deg")

        # 2. Transform to robot coordinates
        try:
            target = self.get_robot_target(obj)
        except RuntimeError as e:
            log(f"Calibration error: {e}")
            return False

        log(f"Robot target: ({target.x:.1f}, {target.y:.1f}, {target.z:.1f})mm")

        # 3. Set speed
        log(f"Setting speed to {self.config.move_speed}%")
        self.robot.set_speed_factor(self.config.move_speed)

        # 4. Open gripper
        log("Opening gripper...")
        self.robot.gripper_move(
            position=self.config.gripper_open_pos,
            speed=self.config.gripper_speed,
            force=self.config.gripper_force,
            wait=True,
        )

        # 5. Move above object
        approach_z = target.z + self.config.approach_height_mm
        log(f"Moving to approach: ({target.x:.1f}, {target.y:.1f}, {approach_z:.1f})mm")
        # Use jog for relative positioning — get current pose first
        current = self.robot.get_cartesian_pose()
        dx = target.x - current[0]
        dy = target.y - current[1]
        dz = approach_z - current[2]
        drz = obj.rotation_deg - current[5]  # align gripper rotation
        self.robot.jog(x=dx, y=dy, z=dz, rz=drz, wait=True)

        # 6. Re-check object position (may have shifted view)
        log("Re-checking object position...")
        time.sleep(0.5)
        obj2 = self.find_object(object_id or obj.id)
        if obj2 is not None:
            target2 = self.get_robot_target(obj2)
            # Small correction if needed
            correction_x = target2.x - target.x
            correction_y = target2.y - target.y
            if abs(correction_x) > 2 or abs(correction_y) > 2:
                log(f"Correcting XY by ({correction_x:.1f}, {correction_y:.1f})mm")
                self.robot.jog(x=correction_x, y=correction_y, wait=True)

        # 7. Descend to grasp height
        descend = -(self.config.approach_height_mm - self.config.grasp_height_mm)
        log(f"Descending {descend:.1f}mm to grasp...")
        self.robot.jog(z=descend, wait=True)

        # 8. Close gripper
        log("Closing gripper...")
        self.robot.gripper_move(
            position=self.config.gripper_close_pos,
            speed=self.config.gripper_speed,
            force=self.config.gripper_force,
            wait=True,
        )
        time.sleep(0.3)  # settle

        # 9. Lift
        log(f"Lifting {self.config.lift_height_mm:.0f}mm...")
        self.robot.jog(z=self.config.lift_height_mm, wait=True)

        # 10. Place (if configured)
        if self.config.place_xyz:
            px, py, pz = self.config.place_xyz
            log(f"Moving to place position: ({px:.1f}, {py:.1f}, {pz:.1f})mm")
            current = self.robot.get_cartesian_pose()
            self.robot.jog(
                x=px - current[0],
                y=py - current[1],
                z=pz - current[2],
                wait=True,
            )
            log("Opening gripper to release...")
            self.robot.gripper_move(
                position=self.config.gripper_open_pos,
                speed=self.config.gripper_speed,
                force=self.config.gripper_force,
                wait=True,
            )

        log("Pick complete!")
        return True
