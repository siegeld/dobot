#!/usr/bin/env python3
"""
Basic usage example for Dobot CR Controller.

This example demonstrates how to:
1. Load configuration
2. Connect to the robot
3. Get current position
4. Access position data
"""

from dobot_cr import DobotController, Config


def main():
    """Main function demonstrating basic usage."""
    # Load configuration from dobot_config.yaml
    print("Loading configuration...")
    config = Config()
    print(f"Robot IP: {config.robot_ip}")
    print()

    # Connect to robot using context manager (auto-disconnect)
    print("Connecting to robot...")
    with DobotController(
        ip=config.robot_ip,
        control_port=config.control_port,
        feedback_port=config.feedback_port,
        timeout=config.timeout,
    ) as robot:
        print("✓ Connected!")
        print()

        # Get current position
        print("Getting robot position...")
        pos = robot.get_position()
        print()

        # Display joint space position
        print("Joint Space Position:")
        for joint, angle in pos.joint_dict.items():
            print(f"  {joint}: {angle:>8.2f}°")
        print()

        # Display cartesian space position
        print("Cartesian Space Position:")
        axes = ["X", "Y", "Z", "RX", "RY", "RZ"]
        for i, (axis, value) in enumerate(pos.cartesian_dict.items()):
            if i < 3:  # X, Y, Z
                print(f"  {axis}:  {value:>8.2f} mm")
            else:  # RX, RY, RZ
                print(f"  {axis}: {value:>8.2f}°")
        print()

        # Access individual values
        print("Individual Values:")
        print(f"  First joint angle (J1): {pos.joint[0]:.2f}°")
        print(f"  X position: {pos.cartesian[0]:.2f} mm")
        print(f"  Y position: {pos.cartesian[1]:.2f} mm")
        print(f"  Z position: {pos.cartesian[2]:.2f} mm")

    print()
    print("✓ Disconnected from robot")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import sys
        sys.exit(1)
