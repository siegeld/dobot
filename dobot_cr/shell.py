"""
Interactive shell for Dobot CR Controller.

Provides a REPL with persistent robot connection, command history,
and emacs-style line editing.
"""

import sys
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

from dobot_cr.config import Config
from dobot_cr.robot import DobotController


# Define prompt style
prompt_style = Style.from_dict({
    'prompt': '#00aa00 bold',
    'robot': '#00aaaa',
})


class DobotShell:
    """Interactive shell for Dobot robot control."""

    def __init__(self, robot: DobotController, config: Config) -> None:
        """
        Initialize the shell.

        Args:
            robot: Connected robot controller
            config: Configuration object
        """
        self.robot = robot
        self.config = config
        self.console = Console()
        self.debug_mode = False  # Debug mode shows angles and confirms moves

        # Command completer
        self.completer = WordCompleter(
            [
                'position',
                'pos',
                'joint',
                'cartesian',
                'cart',
                'enable',
                'disable',
                'clear',
                'remote',
                'status',
                'debug',
                'jog',
                'mode',
                'x', 'y', 'z', 'rx', 'ry', 'rz',  # cartesian jog axes
                'j1', 'j2', 'j3', 'j4', 'j5', 'j6',  # joint jog axes
                'user', 'tool',  # jog modes
                'help',
                'exit',
                'quit',
            ],
            ignore_case=True,
        )

        # Session with history
        self.session: PromptSession = PromptSession(
            history=FileHistory('.dobot_history'),
            auto_suggest=AutoSuggestFromHistory(),
            completer=self.completer,
            style=prompt_style,
        )

    def print_welcome(self) -> None:
        """Print welcome message."""
        welcome = Panel(
            f"[bold cyan]Dobot CR Interactive Shell[/bold cyan]\n\n"
            f"Connected to: [yellow]{self.robot.ip}[/yellow]\n"
            f"Type [green]help[/green] for available commands, "
            f"[green]exit[/green] or [green]quit[/green] to leave\n"
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

        table.add_row("position", "pos", "Show both joint and cartesian position")
        table.add_row("joint", "", "Show joint space position only")
        table.add_row("cartesian", "cart", "Show cartesian space position only")
        table.add_row("jog j1-j6 [deg]", "", "Joint space: jog j1 5 (move J1 by 5°)")
        table.add_row("jog joints <j1..j6>", "", "All joints: jog joints 1 2 0 0 0 -1")
        table.add_row("jog x/y/z [mm]", "", "Cartesian: jog x 10 (move 10mm in X)")
        table.add_row("jog rx/ry/rz [deg]", "", "Cartesian rotation: jog rz 5 (rotate 5°)")
        table.add_row("jog mode <user|tool>", "", "Switch cartesian coordinate frame")
        table.add_row("dance <deg> <count>", "", "Random dance: dance 5 10 (±5° for 10 moves)")
        table.add_row("enable", "", "Enable the robot")
        table.add_row("disable", "", "Disable the robot")
        table.add_row("clear", "", "Clear robot errors")
        table.add_row("remote", "", "Request TCP/IP control (for jog/move)")
        table.add_row("status", "", "Show robot connection status")
        table.add_row("debug", "", "Toggle debug mode (shows angles, confirms moves)")
        table.add_row("help", "?", "Show this help message")
        table.add_row("exit", "quit", "Exit the shell")

        self.console.print(table)

    def cmd_position(self, mode: Optional[str] = None) -> None:
        """Get robot position."""
        try:
            pos = self.robot.get_position()
            precision = self.config.precision

            if mode != "cartesian" and mode != "cart":
                # Joint space
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

                self.console.print(joint_table)

            if mode != "joint":
                # Cartesian space
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

                self.console.print(cart_table)

        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_enable(self) -> None:
        """Enable the robot."""
        try:
            self.robot.enable_robot()
            self.console.print("[bold green]✓[/bold green] Robot enabled")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_disable(self) -> None:
        """Disable the robot."""
        try:
            self.robot.disable_robot()
            self.console.print("[bold green]✓[/bold green] Robot disabled")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_clear(self) -> None:
        """Clear robot errors."""
        try:
            self.robot.clear_error()
            self.console.print("[bold green]✓[/bold green] Errors cleared")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_remote(self) -> None:
        """Request TCP/IP control of the robot."""
        try:
            self.robot.request_control()
            self.console.print("[bold green]✓[/bold green] TCP/IP control requested")
            self.console.print("[yellow]Note:[/yellow] Robot is now in remote control mode")
        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_status(self) -> None:
        """Show connection status."""
        status_table = Table(box=box.ROUNDED)
        status_table.add_column("Status", style="cyan")
        status_table.add_column("Value", style="yellow")

        status_table.add_row("Connected", "Yes" if self.robot.is_connected else "No")
        status_table.add_row("Robot IP", self.robot.ip)
        status_table.add_row("Control Port", str(self.robot.control_port))
        status_table.add_row("Feedback Port", str(self.robot.feedback_port))
        status_table.add_row("Jog Mode", self.robot.jog_mode.capitalize())
        status_table.add_row("Debug Mode", "ON" if self.debug_mode else "OFF")

        self.console.print(status_table)

    def cmd_debug(self) -> None:
        """Toggle debug mode."""
        self.debug_mode = not self.debug_mode
        status = "[bold green]ON[/bold green]" if self.debug_mode else "[bold red]OFF[/bold red]"
        self.console.print(f"Debug mode: {status}")
        if self.debug_mode:
            self.console.print("[yellow]Debug mode will show joint angles and confirm before moves[/yellow]")

    def cmd_dance(self, args: list) -> None:
        """
        Random dance motion.

        Args:
            args: [degrees, count] - max offset and repeat count
        """
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

        # Safety limits
        if degrees > 20:
            self.console.print("[bold red]Error:[/bold red] degrees limited to 20° for safety")
            return
        if degrees < 0:
            self.console.print("[bold red]Error:[/bold red] degrees must be positive")
            return
        if count > 100:
            self.console.print("[bold red]Error:[/bold red] count limited to 100 for safety")
            return
        if count < 1:
            self.console.print("[bold red]Error:[/bold red] count must be at least 1")
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
            import time

            # Save starting position
            start_pos = self.robot.get_joint_angles()
            self.console.print(f"[bold blue]ℹ[/bold blue] Starting dance from current position...")
            self.console.print(f"[dim]Start: {[f'{a:.1f}' for a in start_pos]}[/dim]")

            # Dance loop
            for i in range(count):
                # Generate random offsets from start position
                offsets = [random.uniform(-degrees, degrees) for _ in range(6)]
                target = [float(start_pos[j]) + offsets[j] for j in range(6)]

                self.console.print(f"[bold cyan]→[/bold cyan] Dance move {i+1}/{count}...")
                self.robot.move_joints(target)

                # Small delay between moves
                time.sleep(0.1)

            # Return to start
            self.console.print(f"[bold blue]→[/bold blue] Returning to start position...")
            self.robot.move_joints(list(start_pos))

            self.console.print(f"[bold green]✓[/bold green] Dance complete!")

        except Exception as e:
            self.console.print(f"[bold red]Error:[/bold red] {e}")

    def cmd_jog(self, args: list) -> None:
        """
        Jog robot command.

        Args:
            args: Command arguments [axis, distance] or ['mode', mode_name]
        """
        if not args:
            self.console.print("[bold red]Error:[/bold red] Usage: jog <axis> [distance] or jog mode <user|tool>")
            return

        # Check for mode change
        if args[0] == 'mode':
            if len(args) < 2:
                self.console.print("[bold red]Error:[/bold red] Usage: jog mode <user|tool>")
                return

            mode = args[1].lower()
            if mode not in ('user', 'tool'):
                self.console.print(f"[bold red]Error:[/bold red] Invalid mode '{mode}'. Use 'user' or 'tool'")
                return

            try:
                self.robot.set_jog_mode(mode)
                self.console.print(f"[bold green]✓[/bold green] Jog mode set to: [yellow]{mode}[/yellow]")
            except Exception as e:
                self.console.print(f"[bold red]Error:[/bold red] {e}")
            return

        # Check for all-joints jogging
        if args[0] == 'joints':
            if len(args) != 7:  # 'joints' + 6 offsets
                self.console.print("[bold red]Error:[/bold red] Usage: jog joints <j1> <j2> <j3> <j4> <j5> <j6>")
                self.console.print("Example: jog joints 1 2 0 0 0 -1")
                return

            try:
                offsets = [float(args[i]) for i in range(1, 7)]
            except ValueError:
                self.console.print("[bold red]Error:[/bold red] All joint offsets must be numbers")
                return

            try:
                # Get current angles
                current = self.robot.get_joint_angles()
                target = [float(current[i]) + offsets[i] for i in range(6)]

                # Debug mode: show angles and confirm
                if self.debug_mode:
                    self.console.print(f"\n[bold yellow]DEBUG: Joint Vector Jog[/bold yellow]")
                    self.console.print("\nCurrent angles:")
                    for i, angle in enumerate(current, 1):
                        self.console.print(f"  J{i}: {angle:8.3f}°")

                    self.console.print("\nOffsets:")
                    for i, offset in enumerate(offsets, 1):
                        if abs(offset) > 0.001:
                            self.console.print(f"  J{i}: {offset:+8.3f}°")

                    self.console.print("\nTarget angles:")
                    for i in range(6):
                        delta = target[i] - current[i]
                        if abs(delta) > 0.001:
                            self.console.print(f"  J{i+1}: {target[i]:8.3f}° ([green]{delta:+.3f}°[/green])")
                        else:
                            self.console.print(f"  J{i+1}: {target[i]:8.3f}°")

                    response = self.session.prompt("\n[bold cyan]Execute move? (y/n):[/bold cyan] ")
                    if response.lower() not in ('y', 'yes'):
                        self.console.print("[yellow]Move cancelled[/yellow]")
                        return

                self.console.print(f"[bold blue]→[/bold blue] Jogging all joints (joint space)...")
                self.robot.move_joints(target)
                self.console.print("[bold green]✓[/bold green] Move complete")

            except Exception as e:
                self.console.print(f"[bold red]Error:[/bold red] {e}")
            return

        # Regular jog command
        axis = args[0].lower()
        valid_cartesian_axes = {'x', 'y', 'z', 'rx', 'ry', 'rz'}
        valid_joint_axes = {'j1', 'j2', 'j3', 'j4', 'j5', 'j6'}

        if axis not in valid_cartesian_axes and axis not in valid_joint_axes:
            self.console.print(f"[bold red]Error:[/bold red] Invalid axis '{axis}'. Use: x, y, z, rx, ry, rz or j1, j2, j3, j4, j5, j6")
            return

        # Get distance (use default if not specified)
        if len(args) >= 2:
            try:
                distance = float(args[1])
            except ValueError:
                self.console.print(f"[bold red]Error:[/bold red] Invalid distance '{args[1]}'. Must be a number")
                return
        else:
            # Use config defaults
            if axis in ('x', 'y', 'z'):
                distance = self.config.jog_default_distance
            elif axis in ('rx', 'ry', 'rz'):
                distance = self.config.jog_default_rotation
            else:  # j1-j6
                distance = self.config.jog_default_rotation  # Use same default for joints

        # NOTE: RequestControl() corrupts feedback data (returns garbage values)
        # Try jogging without it - if you get "Control Mode Is Not Tcp" error,
        # we'll need to find another solution
        # if not hasattr(self, '_control_requested'):
        #     self.console.print("[bold yellow]→[/bold yellow] Requesting TCP/IP control...")
        #     try:
        #         self.robot.request_control()
        #         self._control_requested = True
        #         self.console.print("[bold green]✓[/bold green] Control granted")
        #     except Exception as e:
        #         self.console.print(f"[bold red]Error requesting control:[/bold red] {e}")
        #         return

        try:
            # Handle joint vs cartesian jogging
            if axis in valid_joint_axes:
                # Joint jogging - no singularities!
                joint_num = int(axis[1])  # Extract number from 'j1', 'j2', etc.

                # Debug mode: show angles and confirm
                if self.debug_mode:
                    current = self.robot.get_joint_angles()
                    target = [float(x) for x in current]
                    target[joint_num - 1] += distance

                    self.console.print(f"\n[bold yellow]DEBUG: Joint Jog {axis.upper()}{distance:+.1f}°[/bold yellow]")
                    self.console.print("\nCurrent angles:")
                    for i, angle in enumerate(current, 1):
                        marker = " ← WILL CHANGE" if i == joint_num else ""
                        self.console.print(f"  J{i}: {angle:8.3f}°{marker}")

                    self.console.print("\nTarget angles:")
                    for i, angle in enumerate(target, 1):
                        delta = angle - current[i-1]
                        if abs(delta) > 0.001:
                            self.console.print(f"  J{i}: {angle:8.3f}° ([green]{delta:+.3f}°[/green])")
                        else:
                            self.console.print(f"  J{i}: {angle:8.3f}°")

                    response = self.session.prompt("\n[bold cyan]Execute move? (y/n):[/bold cyan] ")
                    if response.lower() not in ('y', 'yes'):
                        self.console.print("[yellow]Move cancelled[/yellow]")
                        return

                self.console.print(
                    f"[bold blue]→[/bold blue] Jogging {axis.upper()}{distance:+.1f}° (joint space)..."
                )
                self.robot.jog_joint(joint_num, distance)
            else:
                # Cartesian jogging
                unit = "mm" if axis in ('x', 'y', 'z') else "°"
                mode_str = self.robot.jog_mode.capitalize()
                self.console.print(
                    f"[bold blue]→[/bold blue] Jogging {axis.upper()}{distance:+.1f}{unit} "
                    f"([yellow]{mode_str}[/yellow] coordinates)..."
                )

                # Build jog parameters
                jog_params = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
                jog_params[axis] = distance

                # Execute cartesian jog (speed controlled by global SpeedFactor)
                self.robot.jog(
                    x=jog_params['x'],
                    y=jog_params['y'],
                    z=jog_params['z'],
                    rx=jog_params['rx'],
                    ry=jog_params['ry'],
                    rz=jog_params['rz'],
                )

            self.console.print("[bold green]✓[/bold green] Jog command sent")

        except Exception as e:
            error_msg = str(e)
            if "Control Mode Is Not Tcp" in error_msg or "Not Tcp" in error_msg:
                self.console.print(
                    f"[bold red]Error:[/bold red] Robot not in TCP/IP control mode\n"
                    f"[yellow]Tip:[/yellow] Run the [green]remote[/green] command first, "
                    f"then try jogging again"
                )
            else:
                self.console.print(f"[bold red]Error:[/bold red] {e}")

    def process_command(self, cmd: str) -> bool:
        """
        Process a command.

        Args:
            cmd: Command string

        Returns:
            False to exit, True to continue
        """
        cmd = cmd.strip().lower()

        if not cmd:
            return True

        # Split command and args
        parts = cmd.split()
        command = parts[0]

        # Exit commands
        if command in ('exit', 'quit', 'q'):
            return False

        # Help
        elif command in ('help', '?'):
            self.print_help()

        # Position commands
        elif command in ('position', 'pos'):
            self.cmd_position()
        elif command == 'joint':
            self.cmd_position(mode='joint')
        elif command in ('cartesian', 'cart'):
            self.cmd_position(mode='cartesian')

        # Jog commands
        elif command == 'jog':
            self.cmd_jog(parts[1:])  # Pass remaining args

        # Robot control
        elif command == 'enable':
            self.cmd_enable()
        elif command == 'disable':
            self.cmd_disable()
        elif command == 'clear':
            self.cmd_clear()
        elif command == 'remote':
            self.cmd_remote()

        # Status
        elif command == 'status':
            self.cmd_status()

        # Debug
        elif command == 'debug':
            self.cmd_debug()

        # Dance
        elif command == 'dance':
            self.cmd_dance(parts[1:])

        # Unknown command
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
                    # Get user input with prompt
                    user_input = self.session.prompt(
                        [('class:prompt', 'dobot'), ('class:robot', '> ')],
                    )

                    # Process command
                    if not self.process_command(user_input):
                        break

                except KeyboardInterrupt:
                    self.console.print("\n[yellow]Press Ctrl+D or type 'exit' to quit[/yellow]")
                    continue

                except EOFError:
                    break

        finally:
            self.console.print("\n[cyan]Goodbye![/cyan]")


def start_shell(config: Config, ip: Optional[str] = None) -> int:
    """
    Start the interactive shell.

    Args:
        config: Configuration object
        ip: Optional IP override

    Returns:
        Exit code
    """
    console = Console()
    robot_ip = ip or config.robot_ip

    console.print(f"[bold blue]ℹ[/bold blue] Connecting to robot at {robot_ip}...")

    try:
        robot = DobotController(
            ip=robot_ip,
            control_port=config.control_port,
            feedback_port=config.feedback_port,
            timeout=config.timeout,
        )
        robot.connect()

        # Set initial jog mode from config
        robot.set_jog_mode(config.jog_coordinate_mode)

        console.print(f"[bold green]✓[/bold green] Connected!\n")

        # Start shell
        shell = DobotShell(robot, config)
        shell.run()

        # Disconnect
        console.print("[bold blue]ℹ[/bold blue] Disconnecting...")
        robot.disconnect()
        console.print("[bold green]✓[/bold green] Disconnected")

        return 0

    except Exception as e:
        console.print(f"[bold red]Error:[/bold red] {e}")
        return 1
