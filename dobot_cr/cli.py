"""
Command-line interface for Dobot CR Controller.
"""

import sys
from pathlib import Path
from typing import Optional

import click
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich import box

from dobot_cr import __version__
from dobot_cr.config import Config
from dobot_cr.robot import DobotController
from dobot_cr.shell import start_shell


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


@click.group(invoke_without_command=True)
@click.version_option(version=__version__, prog_name="dobot-cr")
@click.pass_context
def cli(ctx: click.Context) -> None:
    """
    Professional CLI tool for controlling Dobot CR series collaborative robots.

    When invoked without a command, starts the interactive shell.

    Configure your robot IP in dobot_config.yaml or dobot_config.local.yaml.
    """
    # Load configuration and store in context
    try:
        ctx.ensure_object(dict)
        ctx.obj["config"] = Config()
    except FileNotFoundError as e:
        print_error(str(e))
        sys.exit(1)

    # If no subcommand was provided, start interactive shell
    if ctx.invoked_subcommand is None:
        sys.exit(start_shell(ctx.obj["config"], ip=None))


@cli.command()
@click.option(
    "--ip",
    help="Robot IP address (overrides config file)",
    type=str,
)
@click.pass_context
def connect(ctx: click.Context, ip: Optional[str]) -> None:
    """Test connection to the robot."""
    config: Config = ctx.obj["config"]
    robot_ip = ip or config.robot_ip

    print_info(f"Connecting to robot at {robot_ip}...")

    try:
        with DobotController(
            ip=robot_ip,
            control_port=config.control_port,
            feedback_port=config.feedback_port,
            timeout=config.timeout,
        ) as robot:
            print_success(f"Successfully connected to robot at {robot_ip}")
    except Exception as e:
        print_error(f"Connection failed: {e}")
        sys.exit(1)


@cli.command()
@click.option(
    "--joint",
    "mode",
    flag_value="joint",
    help="Show only joint space position",
)
@click.option(
    "--cartesian",
    "mode",
    flag_value="cartesian",
    help="Show only cartesian space position",
)
@click.option(
    "--ip",
    help="Robot IP address (overrides config file)",
    type=str,
)
@click.option(
    "--format",
    "output_format",
    type=click.Choice(["table", "json", "yaml"], case_sensitive=False),
    help="Output format (overrides config file)",
)
@click.pass_context
def position(
    ctx: click.Context,
    mode: Optional[str],
    ip: Optional[str],
    output_format: Optional[str],
) -> None:
    """
    Report current robot position.

    By default, shows both joint space (angles) and cartesian space (coordinates).
    Use --joint or --cartesian to show only one space.

    Examples:

      dobot-cr position                  # Show both joint and cartesian

      dobot-cr position --joint          # Joint space only

      dobot-cr position --cartesian      # Cartesian space only

      dobot-cr position --format json    # Output as JSON
    """
    config: Config = ctx.obj["config"]
    robot_ip = ip or config.robot_ip
    fmt = output_format or config.output_format
    precision = config.precision

    try:
        with DobotController(
            ip=robot_ip,
            control_port=config.control_port,
            feedback_port=config.feedback_port,
            timeout=config.timeout,
        ) as robot:
            pos = robot.get_position()

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

            else:  # table format
                if mode != "cartesian":
                    # Joint space table
                    joint_table = Table(
                        title="Joint Space Position",
                        box=box.ROUNDED,
                        show_header=True,
                        header_style="bold cyan",
                    )
                    joint_table.add_column("Joint", style="cyan", justify="center")
                    joint_table.add_column("Angle (°)", style="yellow", justify="right")

                    for joint_name, angle in pos.joint_dict.items():
                        joint_table.add_row(joint_name, f"{angle:.{precision}f}")

                    console.print(joint_table)

                if mode != "joint":
                    # Cartesian space table
                    cart_table = Table(
                        title="Cartesian Space Position",
                        box=box.ROUNDED,
                        show_header=True,
                        header_style="bold magenta",
                    )
                    cart_table.add_column("Axis", style="magenta", justify="center")
                    cart_table.add_column("Value", style="green", justify="right")

                    for i, (axis, value) in enumerate(pos.cartesian_dict.items()):
                        if i < 3:  # X, Y, Z in mm
                            cart_table.add_row(axis, f"{value:.{precision}f} mm")
                        else:  # RX, RY, RZ in degrees
                            cart_table.add_row(axis, f"{value:.{precision}f}°")

                    console.print(cart_table)

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

    table.add_row("Robot IP", config.robot_ip)
    table.add_row("Control Port", str(config.control_port))
    table.add_row("Feedback Port", str(config.feedback_port))
    table.add_row("Timeout", f"{config.timeout}s")
    table.add_row("Output Format", config.output_format)
    table.add_row("Precision", str(config.precision))
    table.add_row("Color Output", str(config.use_color))

    console.print(table)


@cli.command()
@click.option(
    "--ip",
    help="Robot IP address (overrides config file)",
    type=str,
)
@click.pass_context
def shell(ctx: click.Context, ip: Optional[str]) -> None:
    """
    Start interactive shell with persistent robot connection.

    The shell provides a REPL environment where you can execute multiple
    commands without reconnecting. Features include:

    - Persistent connection to robot
    - Command history with up/down arrows
    - Emacs-style line editing (Ctrl+A, Ctrl+E, etc.)
    - Tab completion for commands
    - Auto-suggestions from history

    Available commands in shell:
      position, pos       - Show both joint and cartesian position
      joint               - Show joint space position only
      cartesian, cart     - Show cartesian space position only
      enable              - Enable the robot
      disable             - Disable the robot
      clear               - Clear robot errors
      status              - Show connection status
      help, ?             - Show help
      exit, quit          - Exit shell

    Examples:

      dobot-cr shell                 # Start interactive shell

      dobot-cr shell --ip 10.11.6.69 # Connect to specific IP
    """
    config: Config = ctx.obj["config"]
    sys.exit(start_shell(config, ip))


@cli.command()
@click.argument("axis", type=click.Choice(["x", "y", "z", "rx", "ry", "rz"], case_sensitive=False))
@click.argument("distance", type=float)
@click.option(
    "--mode",
    type=click.Choice(["user", "tool"], case_sensitive=False),
    help="Coordinate mode (overrides config)",
)
@click.option(
    "--speed",
    type=int,
    help="Movement speed percentage (1-100, overrides config)",
)
@click.option(
    "--ip",
    help="Robot IP address (overrides config file)",
    type=str,
)
@click.pass_context
def jog(
    ctx: click.Context,
    axis: str,
    distance: float,
    mode: Optional[str],
    speed: Optional[int],
    ip: Optional[str],
) -> None:
    """
    Jog robot by a relative distance (one-shot command).

    AXIS: x, y, z (linear in mm), rx, ry, rz (rotation in degrees)

    DISTANCE: Distance to move (positive or negative)

    Examples:

      dobot-cr jog x 10          # Move +10mm in X (user coords)

      dobot-cr jog z -5          # Move -5mm in Z

      dobot-cr jog rx 15         # Rotate +15° around X-axis

      dobot-cr jog y 20 --mode tool    # Move in tool coordinates

      dobot-cr jog z 5 --speed 30      # Slower movement (30%)
    """
    config: Config = ctx.obj["config"]
    robot_ip = ip or config.robot_ip
    jog_mode = mode or config.jog_coordinate_mode
    jog_speed = speed or config.jog_speed

    # Validate speed
    if jog_speed < 1 or jog_speed > 100:
        print_error("Speed must be between 1 and 100")
        sys.exit(1)

    try:
        with DobotController(
            ip=robot_ip,
            control_port=config.control_port,
            feedback_port=config.feedback_port,
            timeout=config.timeout,
        ) as robot:
            # Set jog mode
            robot.set_jog_mode(jog_mode)

            # Prepare jog parameters
            jog_params = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            jog_params[axis.lower()] = distance

            # Show what we're doing
            unit = "mm" if axis.lower() in ("x", "y", "z") else "°"
            print_info(
                f"Jogging {axis.upper()}{distance:+.1f}{unit} "
                f"({jog_mode.capitalize()} coordinates, {jog_speed}% speed)"
            )

            # Execute jog
            robot.jog(
                x=jog_params["x"],
                y=jog_params["y"],
                z=jog_params["z"],
                rx=jog_params["rx"],
                ry=jog_params["ry"],
                rz=jog_params["rz"],
                speed=jog_speed,
            )

            print_success("Jog command completed")

    except Exception as e:
        print_error(f"Jog failed: {e}")
        sys.exit(1)


@cli.command()
def completion() -> None:
    """
    Generate shell completion script.

    For Bash, add to ~/.bashrc:
      eval "$(_DOBOT_CR_COMPLETE=bash_source dobot-cr)"

    For Zsh, add to ~/.zshrc:
      eval "$(_DOBOT_CR_COMPLETE=zsh_source dobot-cr)"

    For Fish, add to ~/.config/fish/completions/dobot-cr.fish:
      eval (env _DOBOT_CR_COMPLETE=fish_source dobot-cr)
    """
    console.print(
        Panel.fit(
            "[bold]Shell Completion Setup[/bold]\n\n"
            "[cyan]Bash:[/cyan]\n"
            "  eval \"$(_DOBOT_CR_COMPLETE=bash_source dobot-cr)\"\n\n"
            "[cyan]Zsh:[/cyan]\n"
            "  eval \"$(_DOBOT_CR_COMPLETE=zsh_source dobot-cr)\"\n\n"
            "[cyan]Fish:[/cyan]\n"
            "  eval (env _DOBOT_CR_COMPLETE=fish_source dobot-cr)",
            border_style="blue",
        )
    )


def main() -> None:
    """Entry point for the CLI."""
    cli(obj={})


if __name__ == "__main__":
    main()
