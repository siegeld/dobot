"""
Command-line interface for Dobot ROS2 Controller.

Uses ROS2 services from dobot_bringup_v4 for robot control.
"""

import sys
from typing import Optional

import click
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich import box

import rclpy

from dobot_ros import __version__
from dobot_ros.config import Config
from dobot_ros.ros_client import DobotRosClient
from dobot_ros.shell import start_shell


console = Console()


def print_error(message: str) -> None:
    """Print error message in red."""
    console.print(f"[bold red]Error:[/bold red] {message}")


def print_success(message: str) -> None:
    """Print success message in green."""
    console.print(f"[bold green]✓[/bold green] {message}")


def print_info(message: str) -> None:
    """Print info message."""
    console.print(f"[bold blue]ℹ[/bold blue] {message}")


def get_client(config: Config) -> DobotRosClient:
    """Create and return a ROS2 client."""
    return DobotRosClient(
        namespace=config.ros_namespace,
        service_timeout=config.service_timeout,
    )


@click.group(invoke_without_command=True)
@click.version_option(version=__version__, prog_name="dobot-ros")
@click.pass_context
def cli(ctx: click.Context) -> None:
    """
    ROS2 CLI tool for controlling Dobot CR series collaborative robots.

    Uses dobot_bringup_v4 services for communication.
    When invoked without a command, starts the interactive shell.

    PREREQUISITE: dobot_bringup_v4 node must be running:
      ros2 launch dobot_bringup_v4 bringup_v4.launch.py robot_ip:=<IP>
    """
    # Load configuration
    try:
        ctx.ensure_object(dict)
        ctx.obj["config"] = Config()
    except Exception as e:
        print_error(str(e))
        sys.exit(1)

    # Initialize ROS2 if not already
    if not rclpy.ok():
        rclpy.init()

    # If no subcommand, start interactive shell
    if ctx.invoked_subcommand is None:
        sys.exit(start_shell(ctx.obj["config"]))


@cli.command()
@click.pass_context
def connect(ctx: click.Context) -> None:
    """Test connection to dobot_bringup_v4 services."""
    config: Config = ctx.obj["config"]

    print_info("Checking dobot_bringup_v4 services...")

    try:
        client = get_client(config)

        if client.check_connection():
            print_success("dobot_bringup_v4 services are available")

            # Try to get robot mode
            mode = client.get_robot_mode()
            print_info(f"Robot mode: {mode}")
        else:
            print_error("dobot_bringup_v4 services not available")
            print_info("Make sure dobot_bringup_v4 is running:")
            print_info("  ros2 launch dobot_bringup_v4 bringup_v4.launch.py robot_ip:=<IP>")
            sys.exit(1)

        client.shutdown()

    except Exception as e:
        print_error(f"Connection check failed: {e}")
        sys.exit(1)


@cli.command()
@click.option(
    "--joint", "mode", flag_value="joint",
    help="Show only joint space position",
)
@click.option(
    "--cartesian", "mode", flag_value="cartesian",
    help="Show only cartesian space position",
)
@click.option(
    "--format", "output_format",
    type=click.Choice(["table", "json", "yaml"], case_sensitive=False),
    help="Output format",
)
@click.pass_context
def position(
    ctx: click.Context,
    mode: Optional[str],
    output_format: Optional[str],
) -> None:
    """
    Report current robot position.

    Shows both joint and cartesian positions by default.
    """
    config: Config = ctx.obj["config"]
    fmt = output_format or config.output_format
    precision = config.precision

    try:
        client = get_client(config)
        pos = client.get_position()

        if fmt == "json":
            import json
            data = {}
            if mode != "cartesian":
                data["joint"] = pos.joint_dict
            if mode != "joint":
                data["cartesian"] = pos.cartesian_dict
            console.print(json.dumps(data, indent=2))

        elif fmt == "yaml":
            import yaml
            data = {}
            if mode != "cartesian":
                data["joint"] = pos.joint_dict
            if mode != "joint":
                data["cartesian"] = pos.cartesian_dict
            console.print(yaml.dump(data, default_flow_style=False))

        else:  # table
            if mode != "cartesian":
                joint_table = Table(
                    title="Joint Space Position",
                    box=box.ROUNDED,
                    header_style="bold cyan",
                )
                joint_table.add_column("Joint", style="cyan", justify="center")
                joint_table.add_column("Angle (°)", style="yellow", justify="right")

                for name, angle in pos.joint_dict.items():
                    joint_table.add_row(name, f"{angle:.{precision}f}")

                console.print(joint_table)

            if mode != "joint":
                cart_table = Table(
                    title="Cartesian Space Position",
                    box=box.ROUNDED,
                    header_style="bold magenta",
                )
                cart_table.add_column("Axis", style="magenta", justify="center")
                cart_table.add_column("Value", style="green", justify="right")

                for i, (axis, value) in enumerate(pos.cartesian_dict.items()):
                    unit = "mm" if i < 3 else "°"
                    cart_table.add_row(axis, f"{value:.{precision}f} {unit}")

                console.print(cart_table)

        client.shutdown()

    except Exception as e:
        print_error(f"Failed to get position: {e}")
        sys.exit(1)


@cli.command()
@click.pass_context
def config_show(ctx: click.Context) -> None:
    """Show current configuration."""
    config: Config = ctx.obj["config"]

    table = Table(title="Current Configuration", box=box.ROUNDED)
    table.add_column("Setting", style="cyan")
    table.add_column("Value", style="yellow")

    table.add_row("Config File", str(config.config_file or "defaults"))
    table.add_row("ROS Namespace", config.ros_namespace or "(none)")
    table.add_row("Service Timeout", f"{config.service_timeout}s")
    table.add_row("Output Format", config.output_format)
    table.add_row("Precision", str(config.precision))
    table.add_row("Jog Distance", f"{config.jog_default_distance} mm")
    table.add_row("Jog Rotation", f"{config.jog_default_rotation}°")
    table.add_row("Jog Speed", f"{config.jog_speed}%")
    table.add_row("Jog Mode", config.jog_coordinate_mode)
    table.add_row("Sync Mode", "ON (wait)" if config.sync_mode else "OFF (async)")

    console.print(table)


@cli.command()
@click.pass_context
def shell(ctx: click.Context) -> None:
    """
    Start interactive shell with persistent ROS2 connection.

    Features command history, tab completion, and emacs keybindings.
    """
    config: Config = ctx.obj["config"]
    sys.exit(start_shell(config))


@cli.command()
@click.argument("axis", type=click.Choice(
    ["x", "y", "z", "rx", "ry", "rz", "j1", "j2", "j3", "j4", "j5", "j6"],
    case_sensitive=False
))
@click.argument("distance", type=float)
@click.option(
    "--mode", type=click.Choice(["user", "tool"], case_sensitive=False),
    help="Coordinate mode for cartesian jog",
)
@click.option(
    "--speed", type=int,
    help="Movement speed percentage (1-100)",
)
@click.option(
    "--sync/--async", "sync_mode", default=None,
    help="Wait for motion to complete (sync) or return immediately (async)",
)
@click.pass_context
def jog(
    ctx: click.Context,
    axis: str,
    distance: float,
    mode: Optional[str],
    speed: Optional[int],
    sync_mode: Optional[bool],
) -> None:
    """
    Jog robot by a relative distance.

    AXIS: x, y, z, rx, ry, rz (cartesian) or j1-j6 (joint)
    DISTANCE: Distance in mm or degrees

    Examples:
      dobot-ros jog x 10       # Move +10mm in X
      dobot-ros jog j1 5       # Rotate J1 by +5°
      dobot-ros jog z -5 --mode tool
    """
    config: Config = ctx.obj["config"]
    jog_mode = mode or config.jog_coordinate_mode
    jog_speed = speed or config.jog_speed
    wait = sync_mode if sync_mode is not None else config.sync_mode

    if jog_speed < 1 or jog_speed > 100:
        print_error("Speed must be between 1 and 100")
        sys.exit(1)

    try:
        client = get_client(config)

        # Set speed
        client.set_speed_factor(jog_speed)

        axis = axis.lower()
        mode_str = "sync" if wait else "async"

        if axis.startswith('j'):
            # Joint jog
            joint_num = int(axis[1])
            print_info(f"Jogging J{joint_num} by {distance:+.1f}° ({jog_speed}% speed, {mode_str})")
            client.jog_joint(joint_num, distance, wait=wait)
        else:
            # Cartesian jog
            client.set_jog_mode(jog_mode)
            unit = "mm" if axis in ('x', 'y', 'z') else "°"
            print_info(
                f"Jogging {axis.upper()} by {distance:+.1f}{unit} "
                f"({jog_mode} coords, {jog_speed}% speed, {mode_str})"
            )

            jog_params = {'x': 0, 'y': 0, 'z': 0, 'rx': 0, 'ry': 0, 'rz': 0}
            jog_params[axis] = distance
            client.jog(**jog_params, wait=wait)

        if wait:
            print_success("Move complete")
        else:
            print_success("Jog command sent")
        client.shutdown()

    except Exception as e:
        print_error(f"Jog failed: {e}")
        sys.exit(1)


@cli.command()
@click.pass_context
def enable(ctx: click.Context) -> None:
    """Enable the robot."""
    config: Config = ctx.obj["config"]

    try:
        client = get_client(config)
        client.enable_robot()
        print_success("Robot enabled")
        client.shutdown()
    except Exception as e:
        print_error(f"Failed to enable robot: {e}")
        sys.exit(1)


@cli.command()
@click.pass_context
def disable(ctx: click.Context) -> None:
    """Disable the robot."""
    config: Config = ctx.obj["config"]

    try:
        client = get_client(config)
        client.disable_robot()
        print_success("Robot disabled")
        client.shutdown()
    except Exception as e:
        print_error(f"Failed to disable robot: {e}")
        sys.exit(1)


@cli.command()
@click.pass_context
def clear(ctx: click.Context) -> None:
    """Clear robot errors."""
    config: Config = ctx.obj["config"]

    try:
        client = get_client(config)
        client.clear_error()
        print_success("Errors cleared")
        client.shutdown()
    except Exception as e:
        print_error(f"Failed to clear errors: {e}")
        sys.exit(1)


@cli.command()
@click.pass_context
def stop(ctx: click.Context) -> None:
    """Stop robot motion."""
    config: Config = ctx.obj["config"]

    try:
        client = get_client(config)
        client.stop()
        print_success("Robot stopped")
        client.shutdown()
    except Exception as e:
        print_error(f"Failed to stop robot: {e}")
        sys.exit(1)


@cli.command()
def completion() -> None:
    """
    Generate shell completion script.

    For Bash, add to ~/.bashrc:
      eval "$(_DOBOT_ROS_COMPLETE=bash_source dobot-ros)"

    For Zsh, add to ~/.zshrc:
      eval "$(_DOBOT_ROS_COMPLETE=zsh_source dobot-ros)"

    For Fish, add to ~/.config/fish/completions/dobot-ros.fish:
      eval (env _DOBOT_ROS_COMPLETE=fish_source dobot-ros)
    """
    console.print(
        Panel.fit(
            "[bold]Shell Completion Setup[/bold]\n\n"
            "[cyan]Bash:[/cyan]\n"
            "  eval \"$(_DOBOT_ROS_COMPLETE=bash_source dobot-ros)\"\n\n"
            "[cyan]Zsh:[/cyan]\n"
            "  eval \"$(_DOBOT_ROS_COMPLETE=zsh_source dobot-ros)\"\n\n"
            "[cyan]Fish:[/cyan]\n"
            "  eval (env _DOBOT_ROS_COMPLETE=fish_source dobot-ros)",
            border_style="blue",
        )
    )


@cli.group()
def gripper():
    """Control the DH Robotics gripper."""
    pass


@gripper.command('init')
@click.pass_context
def gripper_init(ctx: click.Context) -> None:
    """Initialize the gripper."""
    config: Config = ctx.obj["config"]
    client = ctx.obj.get("client")
    if client is None:
        client = _create_client(config)
        ctx.obj["client"] = client
    try:
        print_info("Initializing gripper...")
        client.gripper_init()
        print_success("Gripper initialized")
    except Exception as e:
        print_error(f"Gripper init failed: {e}")
        sys.exit(1)
    finally:
        client.shutdown()


@gripper.command('open')
@click.option('--speed', '-s', default=50, type=int, help='Speed 1-100 (default 50)')
@click.option('--force', '-f', default=50, type=int, help='Force 20-100 (default 50)')
@click.pass_context
def gripper_open(ctx: click.Context, speed: int, force: int) -> None:
    """Open the gripper."""
    config: Config = ctx.obj["config"]
    client = _create_client(config)
    try:
        client.gripper_init()
        print_info(f"Opening gripper (speed={speed} force={force})...")
        state = client.gripper_open(force=force, speed=speed)
        _print_grip_result(state)
    except Exception as e:
        print_error(str(e))
        sys.exit(1)
    finally:
        client.shutdown()


@gripper.command('close')
@click.option('--speed', '-s', default=50, type=int, help='Speed 1-100 (default 50)')
@click.option('--force', '-f', default=50, type=int, help='Force 20-100 (default 50)')
@click.pass_context
def gripper_close(ctx: click.Context, speed: int, force: int) -> None:
    """Close the gripper."""
    config: Config = ctx.obj["config"]
    client = _create_client(config)
    try:
        client.gripper_init()
        print_info(f"Closing gripper (speed={speed} force={force})...")
        state = client.gripper_close(force=force, speed=speed)
        _print_grip_result(state)
    except Exception as e:
        print_error(str(e))
        sys.exit(1)
    finally:
        client.shutdown()


@gripper.command('move')
@click.argument('position', type=int)
@click.option('--speed', '-s', default=50, type=int, help='Speed 1-100 (default 50)')
@click.option('--force', '-f', default=50, type=int, help='Force 20-100 (default 50)')
@click.pass_context
def gripper_move(ctx: click.Context, position: int, speed: int, force: int) -> None:
    """Move gripper to position (0=closed, 1000=open)."""
    config: Config = ctx.obj["config"]
    client = _create_client(config)
    try:
        client.gripper_init()
        print_info(f"Moving gripper to {position} (speed={speed} force={force})...")
        state = client.gripper_move(position, force=force, speed=speed)
        _print_grip_result(state)
    except Exception as e:
        print_error(str(e))
        sys.exit(1)
    finally:
        client.shutdown()


@gripper.command('dance')
@click.argument('cycles', type=int)
@click.option('--speed', '-s', default=80, type=int, help='Speed 1-100 (default 80)')
@click.option('--force', '-f', default=50, type=int, help='Force 20-100 (default 50)')
@click.pass_context
def gripper_dance(ctx: click.Context, cycles: int, speed: int, force: int) -> None:
    """Open and close the gripper random amounts for CYCLES cycles."""
    import random
    config: Config = ctx.obj["config"]
    client = _create_client(config)
    try:
        client.gripper_init()
        print_info(f"Gripper dance: {cycles} cycles (speed={speed} force={force})")
        for i in range(cycles):
            pos = random.randint(0, 1000)
            console.print(f"  [{i+1}/{cycles}] -> {pos}", style="cyan")
            client.gripper_move(pos, force=force, speed=speed)
        print_success("Dance complete")
    except Exception as e:
        print_error(str(e))
        sys.exit(1)
    finally:
        client.shutdown()


@gripper.command('status')
@click.pass_context
def gripper_status(ctx: click.Context) -> None:
    """Show gripper status."""
    config: Config = ctx.obj["config"]
    client = _create_client(config)
    try:
        client.gripper_connect()
        pos = client.gripper_get_position()
        state = client.gripper_get_state()
        init = client.gripper_get_init_status()
        state_names = {0: 'Moving', 1: 'Reached', 2: 'Object caught', 3: 'Object dropped'}
        table = Table(title="Gripper Status", box=box.ROUNDED)
        table.add_column("Parameter", style="cyan")
        table.add_column("Value", style="yellow")
        table.add_row("Initialized", 'Yes' if init == 1 else 'No')
        table.add_row("Position", f'{pos}/1000' if pos is not None else 'N/A')
        table.add_row("State", state_names.get(state, str(state)) if state is not None else 'N/A')
        console.print(table)
    except Exception as e:
        print_error(str(e))
        sys.exit(1)
    finally:
        client.shutdown()


def _create_client(config: Config) -> DobotRosClient:
    """Create a ROS2 client."""
    if not rclpy.ok():
        rclpy.init()
    return DobotRosClient(
        namespace=config.ros_namespace,
        service_timeout=config.service_timeout,
    )


def _print_grip_result(state: int) -> None:
    """Print gripper result."""
    states = {1: 'Position reached', 2: 'Object caught', 3: 'Object dropped', -1: 'Timeout'}
    msg = states.get(state, f'State={state}')
    if state in (1, 2):
        print_success(msg)
    elif state == -1:
        print_error(msg)
    else:
        print_info(msg)


# --- Vision / Pick Commands ---


@cli.command()
@click.option("--object-id", "-id", type=int, default=None, help="Object ID to pick (default: largest)")
@click.option("--camera-url", default="http://10.11.6.65:8080", help="Camera server URL")
@click.pass_context
def pick(ctx: click.Context, object_id: Optional[int], camera_url: str) -> None:
    """Pick up an object detected by the camera."""
    from dobot_ros.vision import VisionClient
    from dobot_ros.pick import PickExecutor, PickConfig

    config = ctx.obj.get("config", Config())
    client = _create_client(config)
    vision = VisionClient(base_url=camera_url)

    if not vision.is_connected():
        print_error(f"Camera server not reachable at {camera_url}")
        return

    cal = vision.get_calibration_status()
    if not cal.get("solved"):
        print_error("Camera not calibrated. Run 'dobot-ros calibrate' first.")
        return

    executor = PickExecutor(client, vision)
    success = executor.pick(object_id=object_id, log_callback=lambda m: print_info(m))

    if success:
        print_success("Pick completed")
    else:
        print_error("Pick failed")


@cli.command()
@click.option("--camera-url", default="http://10.11.6.65:8080", help="Camera server URL")
@click.pass_context
def scan(ctx: click.Context, camera_url: str) -> None:
    """Show objects detected by the camera."""
    from dobot_ros.vision import VisionClient

    vision = VisionClient(base_url=camera_url)
    if not vision.is_connected():
        print_error(f"Camera not reachable at {camera_url}")
        return

    objects = vision.get_objects()
    if not objects:
        print_info("No objects detected")
        return

    table = Table(title="Detected Objects", box=box.ROUNDED)
    table.add_column("ID", style="cyan")
    table.add_column("Position (m)", style="green")
    table.add_column("Depth", style="yellow")
    table.add_column("Rotation", style="magenta")
    table.add_column("Area", style="dim")
    table.add_column("Tracked", style="dim")

    for obj in objects:
        pos = f"({obj.position_3d[0]:.3f}, {obj.position_3d[1]:.3f}, {obj.position_3d[2]:.3f})"
        table.add_row(
            f"#{obj.id}",
            pos,
            f"{obj.depth_mean:.3f}m",
            f"{obj.rotation_deg:.0f}°",
            str(obj.area_px),
            str(obj.frames_tracked),
        )

    console.print(table)

    cal = vision.get_calibration_status()
    if cal.get("solved"):
        console.print(f"\n[dim]Calibration: solved ({cal['num_points']} pts, err={cal['error_mm']:.1f}mm)[/dim]")
    else:
        console.print(f"\n[yellow]Calibration: NOT solved ({cal['num_points']} pts)[/yellow]")


@cli.group()
def calibrate():
    """Camera-to-robot calibration commands."""
    pass


@calibrate.command("record")
@click.option("--camera-url", default="http://10.11.6.65:8080")
@click.pass_context
def calibrate_record(ctx: click.Context, camera_url: str) -> None:
    """Record a calibration point at the robot's current position.

    Move the robot TCP to a known point visible to the camera,
    then run this command. It records the camera 3D position and
    robot cartesian position as a correspondence pair.
    """
    from dobot_ros.vision import VisionClient

    config = ctx.obj.get("config", Config())
    client = _create_client(config)
    vision = VisionClient(base_url=camera_url)

    # Get robot position
    robot_pose = client.get_cartesian_pose()
    robot_xyz = robot_pose[:3]  # [X, Y, Z] in mm
    print_info(f"Robot position: X={robot_xyz[0]:.1f} Y={robot_xyz[1]:.1f} Z={robot_xyz[2]:.1f} mm")

    # Get camera position at the center of the frame (where the TCP should be visible)
    # The user should have the TCP at a known point visible to the camera
    # We'll use the depth query at the frame center, or let the user specify pixel coords
    depth_data = vision.get_depth_at(320, 180)
    camera_xyz = depth_data["position_3d"]
    print_info(f"Camera position: x={camera_xyz[0]:.4f} y={camera_xyz[1]:.4f} z={camera_xyz[2]:.4f} m")

    if depth_data["distance"] <= 0:
        print_error("No depth at frame center. Move the robot TCP into view.")
        return

    result = vision.add_calibration_point(camera_xyz, robot_xyz)
    print_success(f"Point recorded. Total: {result['total_points']} points")


@calibrate.command("record-at")
@click.argument("px", type=int)
@click.argument("py", type=int)
@click.option("--camera-url", default="http://10.11.6.65:8080")
@click.pass_context
def calibrate_record_at(ctx: click.Context, px: int, py: int, camera_url: str) -> None:
    """Record a calibration point using a specific pixel coordinate.

    Usage: dobot-ros calibrate record-at 310 270
    """
    from dobot_ros.vision import VisionClient

    config = ctx.obj.get("config", Config())
    client = _create_client(config)
    vision = VisionClient(base_url=camera_url)

    robot_pose = client.get_cartesian_pose()
    robot_xyz = robot_pose[:3]
    print_info(f"Robot: X={robot_xyz[0]:.1f} Y={robot_xyz[1]:.1f} Z={robot_xyz[2]:.1f} mm")

    depth_data = vision.get_depth_at(px, py)
    camera_xyz = depth_data["position_3d"]
    print_info(f"Camera at ({px},{py}): x={camera_xyz[0]:.4f} y={camera_xyz[1]:.4f} z={camera_xyz[2]:.4f} m")

    if depth_data["distance"] <= 0:
        print_error(f"No depth data at pixel ({px}, {py})")
        return

    result = vision.add_calibration_point(camera_xyz, robot_xyz)
    print_success(f"Point recorded. Total: {result['total_points']} points")


@calibrate.command("solve")
@click.option("--camera-url", default="http://10.11.6.65:8080")
def calibrate_solve(camera_url: str) -> None:
    """Solve the camera-to-robot transform from recorded points."""
    from dobot_ros.vision import VisionClient

    vision = VisionClient(base_url=camera_url)
    result = vision.solve_calibration()

    if result.get("solved"):
        print_success(f"Calibration solved! {result['num_points']} points, error={result['error_mm']:.1f}mm")
    else:
        print_error(f"Need at least 3 points (have {result['num_points']})")


@calibrate.command("status")
@click.option("--camera-url", default="http://10.11.6.65:8080")
def calibrate_status(camera_url: str) -> None:
    """Show calibration status and points."""
    from dobot_ros.vision import VisionClient

    vision = VisionClient(base_url=camera_url)
    result = vision.get_calibration_status()

    if result.get("solved"):
        print_success(f"Solved: {result['num_points']} points, error={result['error_mm']:.1f}mm")
    else:
        print_info(f"Not solved. {result['num_points']} points recorded.")

    for i, pt in enumerate(result.get("points", [])):
        cam = pt["camera_xyz"]
        rob = pt["robot_xyz"]
        console.print(
            f"  [{i+1}] camera=({cam[0]:.4f}, {cam[1]:.4f}, {cam[2]:.4f})m "
            f"→ robot=({rob[0]:.1f}, {rob[1]:.1f}, {rob[2]:.1f})mm"
        )


@calibrate.command("clear")
@click.option("--camera-url", default="http://10.11.6.65:8080")
def calibrate_clear(camera_url: str) -> None:
    """Clear all calibration points."""
    from dobot_ros.vision import VisionClient

    vision = VisionClient(base_url=camera_url)
    vision.clear_calibration()
    print_success("Calibration cleared")


@calibrate.command("test")
@click.argument("px", type=int)
@click.argument("py", type=int)
@click.option("--camera-url", default="http://10.11.6.65:8080")
@click.pass_context
def calibrate_test(ctx: click.Context, px: int, py: int, camera_url: str) -> None:
    """Test calibration by comparing predicted vs actual robot position.

    Move robot TCP to a point, then: dobot-ros calibrate test 310 270
    """
    from dobot_ros.vision import VisionClient

    config = ctx.obj.get("config", Config())
    client = _create_client(config)
    vision = VisionClient(base_url=camera_url)

    robot_pose = client.get_cartesian_pose()
    actual_xyz = robot_pose[:3]

    depth_data = vision.get_depth_at(px, py)
    if depth_data["distance"] <= 0:
        print_error(f"No depth at ({px}, {py})")
        return

    try:
        predicted = vision.transform_to_robot(depth_data["position_3d"])
    except Exception as e:
        print_error(f"Transform failed: {e}")
        return

    pred = predicted
    dx = pred["robot_xyz"][0] - actual_xyz[0]
    dy = pred["robot_xyz"][1] - actual_xyz[1]
    dz = pred["robot_xyz"][2] - actual_xyz[2]
    import math
    error = math.sqrt(dx*dx + dy*dy + dz*dz)

    console.print(f"  Predicted: ({pred['robot_xyz'][0]:.1f}, {pred['robot_xyz'][1]:.1f}, {pred['robot_xyz'][2]:.1f})mm")
    console.print(f"  Actual:    ({actual_xyz[0]:.1f}, {actual_xyz[1]:.1f}, {actual_xyz[2]:.1f})mm")
    console.print(f"  Error:     {error:.1f}mm (dx={dx:.1f} dy={dy:.1f} dz={dz:.1f})")

    if error < 5:
        print_success(f"Calibration accuracy: {error:.1f}mm — excellent")
    elif error < 15:
        print_info(f"Calibration accuracy: {error:.1f}mm — acceptable")
    else:
        print_error(f"Calibration accuracy: {error:.1f}mm — poor, recalibrate")


@cli.command()
@click.option("--host", default="0.0.0.0", help="Host to bind to")
@click.option("--port", "-p", default=8080, type=int, help="Port to serve on")
@click.pass_context
def web(ctx: click.Context, host: str, port: int) -> None:
    """Start the web dashboard for browser-based robot control."""
    config: Config = ctx.obj["config"]
    print_info(f"Starting web dashboard on http://{host}:{port}")
    from dobot_ros.web.server import start_server
    start_server(config, host=host, port=port)


def main() -> None:
    """Entry point for the CLI."""
    try:
        cli(obj={})
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
