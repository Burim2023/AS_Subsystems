# Command-Based Robot Setup Summary

## What We've Created

### 1. Core Structure Files
- **`RobotContainer.h/cpp`** - Central hub for all subsystems, commands, and button bindings
- **`Robot.h/cpp`** - Updated to use command-based framework properly

### 2. Individual Commands (`src/main/include/commands/` and `src/main/cpp/commands/`)
- **`MoveArmToPosition`** - Moves arm to specific angle, finishes when arm stops moving
- **`MoveGripperToPosition`** - Moves gripper to specific angle, finishes when gripper stops moving
- **`ExtendForDuration`** - Runs extender servo for a specific time duration (clockwise or counter-clockwise)

### 3. Command Groups (Complex Sequences)
- **`FullPickSequence`** - Complete pick sequence:
  1. Open gripper
  2. Move arm to pick position
  3. Extend arm
  4. Close gripper
  5. Wait 0.5 seconds
  6. Retract arm
  7. Move arm to safe position

- **`RetractAndLift`** - Parallel command that retracts extender while moving arm to home (both happen simultaneously)

## How It Works

### Button Bindings (Configured in `RobotContainer::ConfigureButtonBindings()`)
- **A Button** → Move arm to home position
- **B Button** → Run full pick sequence
- **X Button** → Move gripper to up position  
- **Y Button** → Move gripper to down position
- **Right Bumper** → Extend arm (while held, max 5 seconds)
- **Left Bumper** → Retract arm (while held, max 5 seconds)

### Autonomous Mode Selection
- Uses `SendableChooser` on SmartDashboard
- Options:
  - "Full Pick Sequence" (default)
  - "Retract and Lift"
- Selected command runs automatically when autonomous starts

### Key Features
1. **Subsystem Safety**: Commands declare which subsystems they use (`AddRequirements()`), preventing conflicts
2. **Automatic Scheduling**: `CommandScheduler` runs in `RobotPeriodic()`, managing all commands
3. **Reusable Components**: Individual commands can be combined into complex sequences
4. **Dashboard Integration**: Autonomous mode selection via SmartDashboard

## How to Add New Commands

### 1. Create a Simple Command
```cpp
// In commands/MyNewCommand.h
class MyNewCommand : public frc2::CommandHelper<frc2::CommandBase, MyNewCommand> {
public:
    MyNewCommand(MySubsystem* subsystem, parameters...);
    void Initialize() override;     // Called once when command starts
    void Execute() override;        // Called repeatedly while running
    bool IsFinished() override;     // Return true when command should end
    void End(bool interrupted) override; // Called once when command ends
private:
    MySubsystem* m_subsystem;
    // other member variables
};
```

### 2. Create a Command Group
```cpp
// In commands/MySequence.h
class MySequence : public frc2::SequentialCommandGroup {
public:
    MySequence(subsystem pointers...) {
        AddCommands(
            Command1(...),
            Command2(...),
            Command3(...)
        );
    }
};
```

### 3. Add Button Binding
```cpp
// In RobotContainer::ConfigureButtonBindings()
frc2::JoystickButton button(&m_oi.GetDriveController(), buttonNumber);
button.WhenPressed(MyNewCommand(&m_subsystem, parameters...));
```

## Testing Your Setup

1. **Build the project**: `gradlew build`
2. **Test individual commands**: Use button bindings to test each command
3. **Test autonomous**: Use SmartDashboard to select and run autonomous sequences
4. **Debug**: Check console output - all commands print status messages

## Next Steps

To create more complex autonomous routines:
1. Create more command groups combining existing commands
2. Add vision-based commands for apple detection
3. Add drivetrain commands for navigation
4. Create QR code reading commands for dynamic autonomous selection

The command-based framework is now fully set up and ready for your competition tasks!