"""
Driver lifecycle manager for the Dobot ROS2 driver.

Handles waiting for the robot, launching the vendor driver,
detecting disconnection, and auto-reconnecting.
"""

import socket
import subprocess
import signal
import sys
import time


def check_robot(ip: str, port: int = 29999, timeout: float = 2.0) -> bool:
    """Check if the robot's dashboard port is reachable."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except (OSError, socket.timeout):
        return False


def wait_for_robot(ip: str, port: int = 29999, interval: float = 2.0):
    """Block until the robot is reachable."""
    print(f"[driver] Waiting for robot at {ip}:{port} ...")
    while not check_robot(ip, port):
        time.sleep(interval)
    print(f"[driver] Robot reachable at {ip}")


def run_driver(ip: str):
    """Launch, monitor, and auto-reconnect the ROS2 driver."""
    shutting_down = False

    def handle_signal(sig, frame):
        nonlocal shutting_down
        shutting_down = True
        print(f"[driver] Caught signal {sig}, shutting down...")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    while not shutting_down:
        wait_for_robot(ip)

        print("[driver] Launching ROS2 driver...")
        proc = subprocess.Popen(
            ["ros2", "launch", "cr_robot_ros2", "dobot_bringup_ros2.launch.py"],
            stdout=sys.stdout,
            stderr=sys.stderr,
        )

        # Monitor: check robot reachability every 5s
        while proc.poll() is None and not shutting_down:
            time.sleep(5)
            if not check_robot(ip):
                print("[driver] Robot unreachable — stopping driver")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                break

        if shutting_down:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=5)
            break

        print("[driver] Driver exited, retrying in 3s...")
        time.sleep(3)


def main():
    import os
    ip = os.environ.get("IP_address", "192.168.5.1")
    run_driver(ip)


if __name__ == "__main__":
    main()
