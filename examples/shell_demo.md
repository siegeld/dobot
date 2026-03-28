# Interactive Shell Demo

## Starting the Shell

```bash
$ ./dobot-cr.sh shell
ℹ Connecting to robot at 10.11.6.68...
✓ Connected!

╭──────────────────────────────────────────────────────────────╮
│ Dobot CR Interactive Shell                                   │
│                                                              │
│ Connected to: 10.11.6.68                                     │
│ Type help for available commands, exit or quit to leave     │
│ Emacs keybindings enabled (Ctrl+A, Ctrl+E, etc.)           │
╰──────────────────────────────────────────────────────────────╯

dobot>
```

## Example Session

```
dobot> help

           Available Commands
┌────────────┬──────────┬────────────────────────────────────┐
│ Command    │ Aliases  │ Description                        │
├────────────┼──────────┼────────────────────────────────────┤
│ position   │ pos      │ Show both joint and cartesian pos  │
│ joint      │          │ Show joint space position only     │
│ cartesian  │ cart     │ Show cartesian space position only │
│ enable     │          │ Enable the robot                   │
│ disable    │          │ Disable the robot                  │
│ clear      │          │ Clear robot errors                 │
│ status     │          │ Show robot connection status       │
│ help       │ ?        │ Show this help message             │
│ exit       │ quit     │ Exit the shell                     │
└────────────┴──────────┴────────────────────────────────────┘

dobot> position

     Joint Space Position
┌───────┬─────────────┐
│ Joint │  Angle (°)  │
├───────┼─────────────┤
│  J1   │    45.23    │
│  J2   │   -30.45    │
│  J3   │    60.12    │
│  J4   │     0.00    │
│  J5   │    90.00    │
│  J6   │     0.00    │
└───────┴─────────────┘

  Cartesian Space Position
┌──────┬─────────────┐
│ Axis │    Value    │
├──────┼─────────────┤
│  X   │  245.60 mm  │
│  Y   │ -120.30 mm  │
│  Z   │  350.00 mm  │
│  RX  │    0.00°    │
│  RY  │   90.00°    │
│  RZ  │    0.00°    │
└──────┴─────────────┘

dobot> joint

     Joint Space Position
┌───────┬─────────────┐
│ Joint │  Angle (°)  │
├───────┼─────────────┤
│  J1   │    45.23    │
│  J2   │   -30.45    │
│  J3   │    60.12    │
│  J4   │     0.00    │
│  J5   │    90.00    │
│  J6   │     0.00    │
└───────┴─────────────┘

dobot> status

┌───────────────┬─────────┐
│ Status        │ Value   │
├───────────────┼─────────┤
│ Connected     │ Yes     │
│ Robot IP      │ 10.11.6.68 │
│ Control Port  │ 29999   │
│ Feedback Port │ 30004   │
└───────────────┴─────────┘

dobot> exit

Goodbye!
ℹ Disconnecting...
✓ Disconnected
```

## Key Features Demonstrated

### 1. Persistent Connection
- Connect once at startup
- Run multiple commands without reconnecting
- Faster than one-shot commands for multiple operations

### 2. Command History
- Use ↑/↓ arrow keys to navigate history
- Previously run commands are saved
- History persists across sessions (`.dobot_history`)

### 3. Emacs Keybindings
- `Ctrl+A` - Move to start of line
- `Ctrl+E` - Move to end of line
- `Ctrl+K` - Kill (cut) from cursor to end
- `Ctrl+U` - Kill from cursor to start
- `Ctrl+W` - Kill previous word
- `Ctrl+Y` - Yank (paste) killed text
- `Ctrl+L` - Clear screen
- `Ctrl+D` - Exit shell (if line is empty)

### 4. Tab Completion
```
dobot> po<TAB>
# Completes to: position

dobot> ca<TAB>
# Shows: cartesian, cart
```

### 5. Auto-Suggestions
As you type, grayed-out suggestions appear based on history.
Press → (right arrow) to accept the suggestion.

## Comparison: Shell vs One-Shot

### Using Shell (Faster for Multiple Commands)
```bash
$ ./dobot-cr.sh shell
# Connects once
dobot> position
dobot> joint
dobot> cartesian
dobot> exit
# Disconnects once
```

### Using One-Shot (Each Command Connects/Disconnects)
```bash
$ ./dobot-cr.sh position  # Connect, run, disconnect
$ ./dobot-cr.sh position --joint  # Connect, run, disconnect
$ ./dobot-cr.sh position --cartesian  # Connect, run, disconnect
```

**Shell mode is ~3x faster for multiple commands!**
