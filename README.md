# Larry2026

It's Larry! This is [Metuchen Momentum's](https://metuchenmomentum.org/) (a 501(c)(3) non-profit) robot for FIRST: Rebuilt 2026!

## Robot overview

Larry is built with Java as a WPILib project. We use AdvantageKit logging and PathPlanner for our autos.
Larry can be set to `REAL`, `SIM`, and replay modes! Infinitely simulated Larries. Also fully supports SysID!

## Subsystems

- `Drive` (`src/main/java/frc/robot/subsystems/drive`)
  - 4-module swerve drive with odometry + pose estimation.
  - PathPlanner for auto plotting.
  - Gyro fallback logic if the sensor disconnects (because robots also have trust issues).
- `Intake` (`src/main/java/frc/robot/subsystems/intake`)
  - Intake roller for pulling fuel in/out.
  - Pivot control for raising/lowering the intake.
- `Conveyor` (`src/main/java/frc/robot/subsystems/conveyor`)
  - Ball mover as an active floor between intake and shooter.
  - Forward/reverse transport commands.
- `Shooter` (`src/main/java/frc/robot/subsystems/shooter`)
  - Shooter wheel, feeder, and shooter pivot.
  - Supports shoot/feed forward and reverse.
- `Vision` (`src/main/java/frc/robot/subsystems/vision`)
  - Vision interfaces with Drive & Shooter (We use Limelight! PhotonVision is unmodified and based off template).

## Driver controls (from `RobotContainer`)

- Left stick: translation
- Right stick X: rotation
- `LT` / `RT`: intake in / out
- `LB` / `RB`: intake pivot down / up
- D-pad up/down: shoot+feed forward / reverse
- D-pad left/right: shooter pivot up / down
- `X` / `Y`: feeder (into shooter) forward / reverse
- `A` / `B`: active floor (conveyor) forward / reverse (toggle)
