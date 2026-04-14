# Larry2026 Robot Code Documentation

## Purpose
This document organizes the robot code into subsystems and highlights key top-level robot files (`RobotContainer`, `Configs`, `BuildConstants`, `FieldConstants`, and related files) so new developers can navigate the project quickly.

---

## 1) Project Entry and Runtime Flow

### `Main.java`
- JVM entrypoint; starts WPILib robot runtime.

### `Robot.java`
- Owns high-level robot lifecycle (`robotInit`, periodic methods, autonomous/teleop transitions).
- Delegates subsystem wiring and command bindings to `RobotContainer`.

### `RobotContainer.java`
- Central composition root for the command-based architecture.
- Instantiates all subsystems for REAL/SIM/REPLAY modes.
- Chooses proper hardware/sim IO implementations.
- Registers PathPlanner named commands.
- Defines controller mappings and default commands.
- Provides autonomous command chooser.

### `Constants.java`
- Global runtime mode selection (`REAL`, `SIM`, `REPLAY`).
- Global feature flags like `enableVision`.

---

## 2) Configuration and Constant Files

### `Configs.java`
Holds REV Spark MAX configuration objects, grouped by mechanism:
- `IntakeConfig`
- `ShooterConfig`
- `FeederConfig`
- `ConveyorConfig`
- `ClimberConfig`

Each group defines:
- motor idle mode
- voltage compensation
- current limits
- inversion/follower behavior
- encoder conversion factors (where needed)
- closed-loop PID/PIDF and output limits

### `BuildConstants.java`
- **Auto-generated** class (configured in `build.gradle` via the `gversion` plugin).
- Captures build metadata such as git revision, build date/time, branch, and dirty state.
- Useful for dashboards, logs, and post-match traceability.

### `FieldConstants.java`
- Canonical geometric model of the 2026 REBUILT field.
- Stores dimensions, element positions, AprilTag layout, and predefined poses.
- Includes nested groupings such as Hub/Trench/Outpost/Tower/Depot/Bump/StartingPoses.
- Used by autonomous pathing, pose targeting, and alliance-aware logic.

### Subsystem-specific constants files
Each mechanism folder generally contains a dedicated constants file:
- `DriveConstants.java`
- `VisionConstants.java`
- `IntakePivotConstants.java`
- `IntakeFlywheelConstants.java`
- `ShooterFlywheelConstants.java`
- `ShooterPivotConstants.java`
- `FeederConstants.java`
- `ConveyorConstants.java`
- `ClimberConstants.java`

These isolate tunable values from behavior code.

---

## 3) Subsystem Architecture Pattern

Most subsystems follow this layered pattern:
1. **Subsystem class**: high-level behavior, commands, periodic logic.
2. **IO interface**: hardware abstraction contract.
3. **Hardware implementation** (`...IOSpark`, `...IOLimelight`, etc.): real robot devices.
4. **Simulation implementation** (`...IOSim`): physics/sim behavior.
5. **Constants/config**: tuning and IDs.

This pattern enables clean switching between REAL and SIM while preserving command logic.

---

## 4) Subsystems by Folder

## `subsystems/drive`
Core files:
- `Drive.java`: swerve subsystem orchestration, odometry/pose estimation integration.
- `Module.java`: per-wheel module behavior.
- `ModuleIO.java`: drive module hardware contract.
- `ModuleIOSpark.java`: real module motor implementation.
- `ModuleIOSim.java`: simulated module implementation.
- `GyroIO.java`, `GyroIONavX.java`, `GyroIOPigeon2.java`: gyro abstraction + implementations.
- `DriveConstants.java`: module geometry, feedforward/PID values, and kinematics constants.
- `SparkOdometryThread.java`: Spark odometry handling support.

Related commands:
- `commands/DriveCommands.java`
- `commands/AlignToPose.java`

## `subsystems/vision`
Core files:
- `Vision.java`: fuses vision measurements into drivetrain pose estimation.
- `VisionIO.java`: vision device abstraction.
- `VisionIOLimelight.java`: Limelight-backed implementation.
- `VisionIOPhotonVision.java`: PhotonVision implementation.
- `VisionIOPhotonVisionSim.java`: PhotonVision simulation implementation.
- `VisionConstants.java`: camera names/transforms/filtering thresholds.
- `LimelightHelpers.java`: Limelight utility wrappers.

## `subsystems/intake`
Two mechanism groups:

### `intake/intakeFlywheel`
- `IntakeFlywheel.java`
- `IntakeFlywheelIO.java`
- `IntakeFlywheelIOSpark.java`
- `IntakeFlywheelIOSim.java`
- `IntakeFlywheelConstants.java`

### `intake/IntakePivot`
- `IntakePivot.java`
- `IntakePivotIO.java`
- `IntakePivotIOSpark.java`
- `IntakePivotIOSim.java`
- `IntakePivotConstants.java`

## `subsystems/shooter`
Two mechanism groups:

### `shooter/shooterFlywheel`
- `ShooterFlywheel.java`
- `ShooterFlywheelIO.java`
- `ShooterFlywheelIOSpark.java`
- `ShooterFlywheelIOSim.java`
- `ShooterFlywheelConstants.java`

### `shooter/shooterPivot`
- `ShooterPivot.java`
- `ShooterPivotIO.java`
- `ShooterPivotIOSpark.java`
- `ShooterPivotIOSim.java`
- `ShooterPivotConstants.java`

Related command:
- `commands/AutoAimShooter.java`

## `subsystems/feeder`
- `Feeder.java`
- `FeederIO.java`
- `FeederIOSpark.java`
- `FeederIOSim.java`
- `FeederConstants.java`

## `subsystems/conveyor`
- `Conveyor.java`
- `ConveyorIO.java`
- `ConveyorIOSpark.java`
- `ConveyorConstants.java`

## `subsystems/climber`
- `Climber.java`
- `ClimberIO.java`
- `ClimberIOSpark.java`
- `ClimberConstants.java`

Note: `RobotContainer` indicates climber hardware is currently not installed.

## `subsystems/marquee`
- `MarqueeSubsystem.java`
- `MarqueeMessage.java`
- `MarqueeMessageBuilder.java`

Handles sponsor and status text output to external display hardware.

---

## 5) Commands Layer (`src/main/java/frc/robot/commands`)

- `DriveCommands.java`: teleop/manual drive command factories.
- `AlignToPose.java`: pose alignment helper command.
- `AutoAimShooter.java`: shooter auto-aim behavior.

Commands express operator intent while subsystems expose mechanism primitives.

---

## 6) Utilities and Support

### `util/`
- `AllianceFlipUtil.java`: coordinate flipping by alliance.
- `LocalADStarAK.java`: pathfinding utility integration.
- `SparkUtil.java`: helper utilities for Spark devices.

### `energy/`
- `BatteryLogger.java`: battery/system logging helpers.

### `org/metuchenmomentum/marquee/`
- Hardware communications library for marquee display connections and message transport.

---

## 7) Deployment/Autonomous Assets

### `src/main/deploy/pathplanner/`
- `autos/*.auto`: autonomous routines.
- `paths/*.path`: reusable trajectories.
- `navgrid.json`, `settings.json`: planner environment and settings.

These assets are deployed to the roboRIO by the Gradle deploy task.

---

## 8) Suggested New-Developer Reading Order

1. `Constants.java`
2. `Robot.java`
3. `RobotContainer.java`
4. `subsystems/drive/Drive.java`
5. One representative mechanism stack (example: `shooterFlywheel` folder)
6. `Configs.java`
7. `FieldConstants.java`
8. `commands/` package

---

## 9) Quick Responsibility Summary for Requested Files

- **RobotContainer**: wiring hub for subsystems, commands, controls, and autos.
- **Configs**: low-level motor/controller configuration bundles.
- **BuildConstants**: generated build/git metadata.
- **FieldConstants**: field geometry and AprilTag references.
- **Constants**: runtime mode and global feature flags.
