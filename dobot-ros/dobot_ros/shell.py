"""
Interactive shell for Dobot ROS2 Controller.

Provides a REPL with persistent ROS2 connection, command history,
and emacs-style line editing.
"""

from typing import Optional

from prompt_toolkit import PromptSession
from prompt_toolkit.completion import WordCompleter
from prompt_toolkit.history import FileHistory
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
from prompt_toolkit.styles import Style
from rich.console import Console
from rich.table import Table
from rich import box
from rich.panel import Panel

import rclpy

from dobot_ros.config import Config
from dobot_ros.ros_client import DobotRosClient


# Prompt style
prompt_style = Style.from_dict({
    'prompt': '#00aa00 bold',
    'ros': '#ff8800',
})


class DobotShell:
    """Interactive shell for Dobot robot control via ROS2."""

    def __init__(self, client: DobotRosClient, config: Config) -> None:
        """
        Initialize the shell.

        Args:
            client: ROS2 client
            config: Configuration object
        """
        self.client = client
        self.config = config
        self.console = Console()
        self.debug_mode = False

        # Command completer
        self.completer = WordCompleter(
            [
                'position', 'pos', 'joint', 'cartesian', 'cart',
                'jog', 'mode', 'joints',
                'x', 'y', 'z', 'rx', 'ry', 'rz',
                'j1', 'j2', 'j3', 'j4', 'j5', 'j6',
                'user', 'tool',
                'enable', 'disable', 'clear', 'stop',
                'dance', 'status', 'debug',
                'help', 'exit', 'quit',
            ],
            ignore_case=True,
        )

        # Session with history
        self.session: PromptSession = PromptSession(
            history=FileHistory('.dobot_ros_history'),
            auto_suggest=AutoSuggestFromHistory(),
            completer=self.completer,
            style=prompt_style,
        )

    def print_welcome(self) -> None:
        """Print welcome message."""
        welcome = Panel(
            f"[bold cyan]Dobot ROS2 Interactive Shell[/bold cyan]\n\n"
            f"Using ROS2 services from [yellow]dobot_bringup_v4[/yellow]\n"
            f"Type [green]help[/green] for available commands, "
            f"[green]exit[/green] to leave\n"
            f"Emacs keybindings enabled (Ctrl+A, Ctrl+E, etc.)",
            border_style="cyan",
            box=box.ROUNDED,
        )
        self.console.print(welcome)
        self.console.print()

    def print_help(self) -> None:
        """Print help information."""
        table = Table(title="Available Commands", box=box.ROUNDED)
        table.add_column("Command", style="cyan", no_wrap=True)
        table.add_column("Aliases", style="yellow")
        table.add_column("Description", style="white")

        table.add_row("position", "pos", "Show joint and cartesian position")
        table.add_row("joint", "", "Show joint space only")
        table.add_row("cartesian", "cart", "Show cartesian space only")
        table.add_row("jog j1-j6 [deg]", "", "Joint jog: jog j1 5")
        table.add_row("jog joints <j1..j6>", "", "All joints: jog joints 1 2 0 0 0 -1")
        table.add_row("jog x/y/z [mm]", "", "Cartesian: jog x 10")
        table.add_row("jog rx/ry/rz [deg]", "", "Rotation: jog rz 5")
        table.add_row("jog mode <user|tool>", "", "Switch coordinate frame")
        table.add_row("dance <deg> <count>", "", "Random dance motion")
        table.add_row("enable", "", "Enable the robot")
        table.add_row("disable", "", "Disable the robot")
        table.add_row("clear", "", "Clear robot errors")
        table.add_row("stop", "", "Stop robot motion")
        table.add_row("status", "", "Show connection status")
        table.add_row("debug", "", "Toggle debug mode")
        table.add_row("help", "?", "Show this help")
        table.add_row("exit", "quit", "Exit the shell")

        self.console.print(table)

    def cmd_position(self, mode: Optional[str] = None) -> None:
        """Get robot position."""
        try:
            pos = self.client.get_position()
            precision = self.config.precision

            if mode != "cartesian" and mode != "cart":
                joint_table = Table(
                    title="Joint Space Position",
                    box=box.ROUNDED,
                    header_style="bold cyan",
                )
                joint_table.add_column("Joint", style="cyan", justify="center")
                joint_table.add_column("Angle (°)", style="yellow", justify="right")

                for name, angle in pos.joint_dict.items():
                    joint_table.add_row(name, f"{angle:.{precision}f}")

                self.console.print(joint_table)

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

                self.console.print(cart_table)

        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_enable(self) -> None:
        """Enable the robot."""
        try:
            self.client.enable_robot()
            self.console.print("[bold green]✓[/bold green] Robot enabled")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_disable(self) -> None:
        """Disable the robot."""
        try:
            self.client.disable_robot()
            self.console.print("[bold green]✓[/bold green] Robot disabled")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_clear(self) -> None:
        """Clear robot errors."""
        try:
            self.client.clear_error()
            self.console.print("[bold green]✓[/bold green] Errors cleared")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_stop(self) -> None:
        """Stop robot motion."""
        try:
            self.client.stop()
            self.console.print("[bold green]✓[/bold green] Robot stopped")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_status(self) -> None:
        """Show connection status."""
        table = Table(box=box.ROUNDED)
        table.add_column("Status", style="cyan")
        table.add_column("Value", style="yellow")

        connected = self.client.check_connection()
        table.add_row("ROS2 Services", "Available" if connected else "Not Available")
        table.add_row("Namespace", self.client.namespace or "(none)")
        table.add_row("Jog Mode", self.client.jog_mode.capitalize())
        table.add_row("Debug Mode", "ON" if self.debug_mode else "OFF")

        try:
            mode = self.client.get_robot_mode()
            table.add_row("Robot Mode", str(mode))
        except:
            table.add_row("Robot Mode", "Unknown")

        self.console.print(table)

    def cmd_debug(self) -> None:
        """Toggle debug mode."""
        self.debug_mode = not self.debug_mode
        status = "[bold green]ON[/bold green]" if self.debug_mode else "[bold red]OFF[/bold red]"
        self.console.print(f"Debug mode: {status}")
        if self.debug_mode:
            self.console.print("[yellow]Debug mode will show joint angles and confirm before moves[/yellow]")

    def cmd_dance(self, args: list) -> None:
        """Random dance motion."""
        if len(args) != 2:
            self.console.print("[bold red]Error:[/bold red] Usage: dance <degrees> <count>")
            self.console.print("Example: dance 5 10 (dance 10 times with ±5° offsets)")
            return

        try:
            degrees = float(args[0])
            count = int(args[1])
        except ValueError:
            self.console.print("[bold red]Error:[/bold red] degrees must be a number, count must be an integer")
            return

        if degrees > 20 or degrees < 0:
            self.console.print("[bold red]Error:[/bold red] degrees limited to 0-20 for safety")
            return
        if count > 100 or count < 1:
            self.console.print("[bold red]Error:[/bold red] count must be 1-100")
            return

        # Confirmation in debug mode
        if self.debug_mode:
            self.console.print(f"\n[bold yellow]DEBUG: Dance Command[/bold yellow]")
            self.console.print(f"  Max offset: ±{degrees}°")
            self.console.print(f"  Iterations: {count}")
            self.console.print(f"  Will return to start position at end")
            response = self.session.prompt("\n[bold cyan]Execute dance? (y/n):[/bold cyan] ")
            if response.lower() not in ('y', 'yes'):
                self.console.print("[yellow]Dance cancelled[/yellow]")
                return

        try:
            import random

            start_pos = self.client.get_joint_angles()
            self.console.print(f"[bold blue]ℹ[/bold blue] Starting dance from current position...")
            self.console.print(f"[dim]Start: {[f'{a:.1f}' for a in start_pos]}[/dim]")

            for i in range(count):
                offsets = [random.uniform(-degrees, degrees) for _ in range(6)]
                target = [start_pos[j] + offsets[j] for j in range(6)]

                self.console.print(f"[bold cyan]→[/bold cyan] Dance move {i+1}/{count}...")
                self.client.move_joints(target, wait=True)

            self.console.print(f"[bold blue]→[/bold blue] Returning to start position...")
            self.client.move_joints(start_pos, wait=True)
            self.console.print(f"[bold green]✓[/bold green] Dance complete!")

        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_jog(self, args: list) -> None:
        """Jog robot command."""
        if not args:
            self.console.print("[bold red]Error:[/bold red] Usage: jog <axis> [distance]")
            return

        # Mode change
        if args[0] == 'mode':
            if len(args) < 2:
                self.console.print("[bold red]Error:[/bold red] Usage: jog mode <user|tool>")
                return
            mode = args[1].lower()
            if mode not in ('user', 'tool'):
                self.console.print(f"[bold red]Error:[/bold red] Invalid mode '{mode}'")
                return
            self.client.set_jog_mode(mode)
            self.console.print(f"[bold green]✓[/bold green] Jog mode: [yellow]{mode}[/yellow]")
            return

        # All-joints jog
        if args[0] == 'joints':
            if len(args) != 7:
                self.console.print("[bold red]Error:[/bold red] Usage: jog joints <j1> <j2> <j3> <j4> <j5> <j6>")
                return
            try:
                offsets = [float(args[i]) for i in range(1, 7)]
                current = self.client.get_joint_angles()
                target = [current[i] + offsets[i] for i in range(6)]

                # Debug mode confirmation
                if self.debug_mode:
                    self.console.print(f"\n[bold yellow]DEBUG: Jog All Joints[/bold yellow]")
                    self.console.print(f"  Current: {[f'{a:.2f}' for a in current]}")
                    self.console.print(f"  Offsets: {[f'{o:+.2f}' for o in offsets]}")
                    self.console.print(f"  Target:  {[f'{t:.2f}' for t in target]}")
                    response = self.session.prompt("\n[bold cyan]Execute move? (y/n):[/bold cyan] ")
                    if response.lower() not in ('y', 'yes'):
                        self.console.print("[yellow]Move cancelled[/yellow]")
                        return

                self.console.print(f"[bold blue]→[/bold blue] Jogging all joints...")
                self.client.move_joints(target)
                self.console.print("[bold green]✓[/bold green] Move complete")
            except Exception as e:
                self.console.print(f"[bold red]Error:[/bold red] {e}")
            return

        # Regular jog
        axis = args[0].lower()
        valid_cartesian = {'x', 'y', 'z', 'rx', 'ry', 'rz'}
        valid_joint = {'j1', 'j2', 'j3', 'j4', 'j5', 'j6'}

        if axis not in valid_cartesian and axis not in valid_joint:
            self.console.print(f"[bold red]Error:[/bold red] Invalid axis '{axis}'")
            return

        # Get distance
        if len(args) >= 2:
            try:
                distance = float(args[1])
            except ValueError:
                self.console.print(f"[bold red]Error:[/bold red] Invalid distance '{args[1]}'")
                return
        else:
            if axis in ('x', 'y', 'z'):
                distance = self.config.jog_default_distance
            else:
                distance = self.config.jog_default_rotation

        try:
            if axis in valid_joint:
                joint_num = int(axis[1])

                # Debug mode confirmation for joint jog
                if self.debug_mode:
                    current = self.client.get_joint_angles()
                    current_angle = current[joint_num - 1]
                    target_angle = current_angle + distance
                    self.console.print(f"\n[bold yellow]DEBUG: Single Joint Jog[/bold yellow]")
                    self.console.print(f"  Joint:   J{joint_num}")
                    self.console.print(f"  Current: {current_angle:.2f}°")
                    self.console.print(f"  Offset:  {distance:+.2f}°")
                    self.console.print(f"  Target:  {target_angle:.2f}°")
                    response = self.session.prompt("\n[bold cyan]Execute move? (y/n):[/bold cyan] ")
                    if response.lower() not in ('y', 'yes'):
                        self.console.print("[yellow]Move cancelled[/yellow]")
                        return

                self.console.print(f"[bold blue]→[/bold blue] Jogging J{joint_num} by {distance:+.1f}°...")
                self.client.jog_joint(joint_num, distance)
            else:
                unit = "mm" if axis in ('x', 'y', 'z') else "°"
                mode_str = self.client.jog_mode.capitalize()

                # Debug mode confirmation for cartesian jog
                if self.debug_mode:
                    pos = self.client.get_position()
                    axis_idx = ['x', 'y', 'z', 'rx', 'ry', 'rz'].index(axis)
                    current_val = pos.cartesian[axis_idx]
                    target_val = current_val + distance
                    self.console.print(f"\n[bold yellow]DEBUG: Cartesian Jog ({mode_str} frame)[/bold yellow]")
                    self.console.print(f"  Axis:    {axis.upper()}")
                    self.console.print(f"  Current: {current_val:.2f}{unit}")
                    self.console.print(f"  Offset:  {distance:+.2f}{unit}")
                    self.console.print(f"  Target:  {target_val:.2f}{unit}")
                    response = self.session.prompt("\n[bold cyan]Execute move? (y/n):[/bold cyan] ")
                    if response.lower() not in ('y', 'yes'):
                        self.console.print("[yellow]Move cancelled[/yellow]")
                        return

                self.console.print(
                    f"[bold blue]→[/bold blue] Jogging {axis.upper()} by {distance:+.1f}{unit} "
                    f"([yellow]{mode_str}[/yellow])..."
                )
                jog_params = {'x': 0, 'y': 0, 'z': 0, 'rx': 0, 'ry': 0, 'rz': 0}
                jog_params[axis] = distance
                self.client.jog(**jog_params)

            self.console.print("[bold green]✓[/bold green] Jog command sent")

        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def process_command(self, cmd: str) -> bool:
        """Process a command. Returns False to exit."""
        cmd = cmd.strip().lower()

        if not cmd:
            return True

        parts = cmd.split()
        command = parts[0]

        if command in ('exit', 'quit', 'q'):
            return False
        elif command in ('help', '?'):
            self.print_help()
        elif command in ('position', 'pos'):
            self.cmd_position()
        elif command == 'joint':
            self.cmd_position(mode='joint')
        elif command in ('cartesian', 'cart'):
            self.cmd_position(mode='cartesian')
        elif command == 'jog':
            self.cmd_jog(parts[1:])
        elif command == 'enable':
            self.cmd_enable()
        elif command == 'disable':
            self.cmd_disable()
        elif command == 'clear':
            self.cmd_clear()
        elif command == 'stop':
            self.cmd_stop()
        elif command == 'status':
            self.cmd_status()
        elif command == 'debug':
            self.cmd_debug()
        elif command == 'dance':
            self.cmd_dance(parts[1:])
        else:
            self.console.print(
                f"[bold red]Unknown command:[/bold red] {command}\n"
                f"Type [green]help[/green] for available commands"
            )

        return True

    def run(self) -> None:
        """Run the interactive shell."""
        self.print_welcome()

        try:
            while True:
                try:
                    user_input = self.session.prompt(
                        [('class:prompt', 'dobot'), ('class:ros', '-ros'), ('', '> ')],
                    )
                    if not self.process_command(user_input):
                        break
                except KeyboardInterrupt:
                    self.console.print("\n[yellow]Press Ctrl+D or type 'exit' to quit[/yellow]")
                except EOFError:
                    break
        finally:
            self.console.print("\n[cyan]Goodbye![/cyan]")


def start_shell(config: Config) -> int:
    """Start the interactive shell."""
    console = Console()

    console.print(f"[bold blue]ℹ[/bold blue] Connecting to dobot_bringup_v4 services...")

    try:
        if not rclpy.ok():
            rclpy.init()

        client = DobotRosClient(
            namespace=config.ros_namespace,
            service_timeout=config.service_timeout,
        )

        if not client.check_connection():
            console.print("[bold red]Error:[/bold red] dobot_bringup_v4 services not available")
            console.print("[yellow]Make sure the bringup node is running:[/yellow]")
            console.print("  ros2 launch dobot_bringup_v4 bringup_v4.launch.py robot_ip:=<IP>")
            return 1

        client.set_jog_mode(config.jog_coordinate_mode)
        console.print(f"[bold green]✓[/bold green] Connected!\n")

        shell = DobotShell(client, config)
        shell.run()

        client.shutdown()
        return 0

    except Exception as e:
        console.print(f"[bold red]Error:[/bold red] {e}")
        return 1
