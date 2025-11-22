#!/usr/bin/env python3
"""
Advanced control example for Dobot CR Controller.

This example demonstrates:
1. Manual connection management
2. Error handling
3. Multiple position queries
4. Robot enable/disable
"""

import time
from dobot_cr import DobotController, Config


def main():
    """Main function demonstrating advanced usage."""
    # Load configuration
    config = Config()

    # Create controller instance
    robot = DobotController(
        ip=config.robot_ip,
        control_port=config.control_port,
        feedback_port=config.feedback_port,
        timeout=config.timeout,
    )

    try:
        # Manual connection
        print("Connecting to robot...")
        robot.connect()
        print(f"✓ Connected to robot at {config.robot_ip}")
        print()

        # Check connection status
        if robot.is_connected:
            print("Robot connection status: OK")
        print()

        # Monitor position continuously
        print("Monitoring robot position (5 iterations)...")
        print("=" * 60)

        for i in range(5):
            try:
                pos = robot.get_position()

                print(f"\nIteration {i + 1}:")
                print(f"  Joint: [{', '.join(f'{j:6.2f}' for j in pos.joint)}]")
                print(
                    f"  Cart:  [{', '.join(f'{c:7.2f}' for c in pos.cartesian)}]"
                )

                # Wait before next reading
                time.sleep(1)

            except Exception as e:
                print(f"  ❌ Error getting position: {e}")
                continue

        print("=" * 60)
        print()

        # Example: Clear any errors (if needed)
        try:
            print("Clearing any robot errors...")
            robot.clear_error()
            print("✓ Errors cleared")
        except Exception as e:
            print(f"⚠️  Could not clear errors: {e}")

    except ConnectionError as e:
        print(f"❌ Connection failed: {e}")
        return 1

    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return 1

    finally:
        # Always disconnect
        print()
        print("Disconnecting from robot...")
        robot.disconnect()
        print("✓ Disconnected")

    return 0


if __name__ == "__main__":
    import sys

    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        sys.exit(130)
