#!/usr/bin/env python3
"""
Jog example for Dobot CR Controller.

This example demonstrates how to:
1. Set jog coordinate mode (user vs tool)
2. Perform incremental jog movements
3. Combine multiple jog operations
"""

import time
from dobot_cr import DobotController, Config


def main():
    """Main function demonstrating jog operations."""
    # Load configuration
    print("Loading configuration...")
    config = Config()
    print(f"Robot IP: {config.robot_ip}")
    print(f"Default jog distance: {config.jog_default_distance}mm")
    print(f"Default jog rotation: {config.jog_default_rotation}°")
    print(f"Jog speed: {config.jog_speed}%")
    print(f"Default coordinate mode: {config.jog_coordinate_mode}")
    print()

    # Connect to robot
    print("Connecting to robot...")
    with DobotController(
        ip=config.robot_ip,
        control_port=config.control_port,
        feedback_port=config.feedback_port,
        timeout=config.timeout,
    ) as robot:
        print("✓ Connected!")
        print()

        # Get initial position
        print("Initial position:")
        pos = robot.get_position()
        print(f"  X: {pos.cartesian[0]:.2f}mm")
        print(f"  Y: {pos.cartesian[1]:.2f}mm")
        print(f"  Z: {pos.cartesian[2]:.2f}mm")
        print()

        # Example 1: User coordinate jog (absolute world axes)
        print("=" * 60)
        print("Example 1: Jogging in USER coordinates")
        print("=" * 60)
        robot.set_jog_mode("user")
        print(f"Jog mode: {robot.jog_mode}")

        # Move 10mm in +X direction
        print("\nMoving +10mm in X (forward)...")
        robot.jog(x=10.0, speed=config.jog_speed)
        time.sleep(1)  # Wait for movement

        # Move 10mm in +Y direction
        print("Moving +10mm in Y (left)...")
        robot.jog(y=10.0, speed=config.jog_speed)
        time.sleep(1)

        # Move 10mm in +Z direction
        print("Moving +10mm in Z (up)...")
        robot.jog(z=10.0, speed=config.jog_speed)
        time.sleep(1)

        # Check new position
        pos = robot.get_position()
        print(f"\nCurrent position after user jog:")
        print(f"  X: {pos.cartesian[0]:.2f}mm")
        print(f"  Y: {pos.cartesian[1]:.2f}mm")
        print(f"  Z: {pos.cartesian[2]:.2f}mm")
        print()

        # Example 2: Return to original position (negative jog)
        print("=" * 60)
        print("Example 2: Returning to original position")
        print("=" * 60)

        print("Moving -10mm in Z (down)...")
        robot.jog(z=-10.0, speed=config.jog_speed)
        time.sleep(1)

        print("Moving -10mm in Y (right)...")
        robot.jog(y=-10.0, speed=config.jog_speed)
        time.sleep(1)

        print("Moving -10mm in X (backward)...")
        robot.jog(x=-10.0, speed=config.jog_speed)
        time.sleep(1)

        # Check final position
        pos = robot.get_position()
        print(f"\nFinal position:")
        print(f"  X: {pos.cartesian[0]:.2f}mm")
        print(f"  Y: {pos.cartesian[1]:.2f}mm")
        print(f"  Z: {pos.cartesian[2]:.2f}mm")
        print()

        # Example 3: Rotation jog
        print("=" * 60)
        print("Example 3: Rotation jog")
        print("=" * 60)

        print("\nRotating +5° around Z-axis...")
        robot.jog(rz=5.0, speed=config.jog_speed)
        time.sleep(1)

        print("Rotating -5° around Z-axis (return)...")
        robot.jog(rz=-5.0, speed=config.jog_speed)
        time.sleep(1)
        print()

        # Example 4: Tool coordinate jog
        print("=" * 60)
        print("Example 4: Jogging in TOOL coordinates")
        print("=" * 60)
        robot.set_jog_mode("tool")
        print(f"Jog mode: {robot.jog_mode}")

        print("\nMoving +10mm along tool X-axis...")
        robot.jog(x=10.0, speed=config.jog_speed)
        time.sleep(1)

        print("Moving -10mm along tool X-axis (return)...")
        robot.jog(x=-10.0, speed=config.jog_speed)
        time.sleep(1)

        # Switch back to user mode
        robot.set_jog_mode("user")
        print(f"\nSwitched back to {robot.jog_mode} mode")
        print()

        # Example 5: Combined movement (diagonal)
        print("=" * 60)
        print("Example 5: Combined movement (diagonal)")
        print("=" * 60)

        print("\nMoving diagonally (+5mm X, +5mm Y simultaneously)...")
        robot.jog(x=5.0, y=5.0, speed=config.jog_speed)
        time.sleep(1)

        print("Returning diagonally (-5mm X, -5mm Y)...")
        robot.jog(x=-5.0, y=-5.0, speed=config.jog_speed)
        time.sleep(1)
        print()

        # Example 6: Slower/faster movement
        print("=" * 60)
        print("Example 6: Speed control")
        print("=" * 60)

        print("\nSlow movement (20% speed): +10mm Z...")
        robot.jog(z=10.0, speed=20)
        time.sleep(2)  # Slower, so wait longer

        print("Fast movement (80% speed): -10mm Z...")
        robot.jog(z=-10.0, speed=80)
        time.sleep(1)
        print()

        print("=" * 60)
        print("✓ All jog examples completed successfully!")
        print("=" * 60)

    print("\n✓ Disconnected from robot")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import sys
        sys.exit(1)
