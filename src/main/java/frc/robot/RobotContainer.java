// Copyright 2021-2025 Team 7587 Metuchen Momentum
// https://github.com/frc-team7587
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAimShooter;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  // private final Vision vision;
  private final Drive drive;
  private final Vision vision;
  private final Intake intake = new Intake(new IntakeIOSpark());
  private final Shooter shooter = new Shooter(new ShooterIOSpark());
  private final Feeder feeder = new Feeder(new FeederIOSpark());
  private final Conveyor conveyor = new Conveyor(new ConveyorIOSpark());
  private final Climber climber = new Climber(new ClimberIOSpark());

  // Input devices
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick keyboard = new Joystick(1);
  double speed = keyboard.getRawAxis(1); // Mapped to W/S
  double turn = keyboard.getRawAxis(4); // Mapped to A/D

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private static final String shooterDashboardTargetRpmKey = "Shooter/DashboardTargetRpm";
  private static final String shooterDashboardOutputKey = "Shooter/DashboardMappedOutput";

  private static final PathConstraints hubPathfindConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private static Vision createVision(Drive drive) {
    if (!Constants.enableVision) {
      return new Vision(drive::addVisionMeasurement, new VisionIO() {});
    }

    switch (Constants.currentMode) {
      case REAL:
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));
      case SIM:
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
      default:
        return new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision = createVision(drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision = createVision(drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = createVision(drive);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    SmartDashboard.putNumber(
        shooterDashboardTargetRpmKey, ShooterConstants.Control.kDashboardDefaultTargetRpm);
    SmartDashboard.putNumber(shooterDashboardOutputKey, 0.0);
    SmartDashboard.putNumber("Shooter/MeasuredVelocityRpm", 0.0);
    SmartDashboard.putNumber("Shooter/CurrentTargetVelocityRpm", 0.0);
    SmartDashboard.putNumber("Shooter/PivotEncoderPosition", 0.0);

    NamedCommands.registerCommand(
        "shoot preload",
        Commands.parallel(
            new AutoAimShooter(drive, vision, shooter, feeder),
            Commands.waitSeconds(0.8).andThen(feeder.feedFuel())));
    NamedCommands.registerCommand("active floor", conveyor.transportBalls());

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Shooter Wheel SysId (Quasistatic Forward)",
    //     shooter.wheelSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Shooter Wheel SysId (Quasistatic Reverse)",
    //     shooter.wheelSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Shooter Wheel SysId (Dynamic Forward)",
    //     shooter.wheelSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Shooter Wheel SysId (Dynamic Reverse)",
    //     shooter.wheelSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Feeder SysId (Quasistatic Forward)",
    //     feeder.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Feeder SysId (Quasistatic Reverse)",
    //     feeder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Feeder SysId (Dynamic Forward)", feeder.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Feeder SysId (Dynamic Reverse)", feeder.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Shooter Pivot SysId (Quasistatic Forward)",
    //     shooter.pivotSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Shooter Pivot SysId (Quasistatic Reverse)",
    //     shooter.pivotSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Shooter Pivot SysId (Dynamic Forward)",
    //     shooter.pivotSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Shooter Pivot SysId (Dynamic Reverse)",
    //     shooter.pivotSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Intake Roller SysId (Quasistatic Forward)",
    //     intake.rollerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Intake Roller SysId (Quasistatic Reverse)",
    //     intake.rollerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Intake Roller SysId (Dynamic Forward)",
    //     intake.rollerSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Intake Roller SysId (Dynamic Reverse)",
    //     intake.rollerSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Intake Pivot SysId (Quasistatic Forward)",
    //     intake.pivotSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Intake Pivot SysId (Quasistatic Reverse)",
    //     intake.pivotSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Intake Pivot SysId (Dynamic Forward)",
    //     intake.pivotSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Intake Pivot SysId (Dynamic Reverse)",
    //     intake.pivotSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Conveyor SysId (Quasistatic Forward)",
    //     conveyor.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Conveyor SysId (Quasistatic Reverse)",
    //     conveyor.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Conveyor SysId (Dynamic Forward)",
    // conveyor.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Conveyor SysId (Dynamic Reverse)",
    // conveyor.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Climber SysId (Quasistatic Forward)",
    //     climber.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Climber SysId (Quasistatic Reverse)",
    //     climber.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Climber SysId (Dynamic Forward)",
    // climber.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Climber SysId (Dynamic Reverse)",
    // climber.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("AutoAim Interpolation Sweep (Sim)", autoAimInterpolationSweep());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Hold X for temporary robot-relative drive.
    // controller
    //     .x()
    //     .whileTrue(
    //         DriveCommands.joystickDriveRobotRelative(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> -controller.getRightX()));

    // TESTING BINDS
    // controller.leftTrigger().toggleOnTrue(intake.intakeFuel());
    // controller.rightTrigger().toggleOnTrue(intake.outtakeFuel());
    // controller.leftBumper().whileTrue(intake.turntoDown());
    // controller.rightBumper().whileTrue(intake.turntoUp());

    // controller
    //     .povUp()
    //     .whileTrue(
    //         Commands.parallel(
    //             shooter.shootFuel(),
    //             Commands.waitUntil(shooter::atRPM).andThen(feeder.feedFuel())));
    // controller
    //     .povDown()
    //     .whileTrue(Commands.parallel(shooter.shootFuelReverse(), feeder.feedFuelReverse()));
    // controller.povLeft().whileTrue(shooter.pivotShooterUp());
    // controller.povRight().whileTrue(shooter.pivotShooterDown());
    // controller.x().whileTrue(feeder.feedFuel());
    // controller.y().whileTrue(feeder.feedFuelReverse());
    // controller.a().toggleOnTrue(conveyor.transportBalls());
    // controller.b().toggleOnTrue(conveyor.transportBallsReverse());
    // controller.back().whileTrue(new AutoAimShooter(drive, vision, shooter));

    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.parallel(
    //             shooter.shootFuel(),
    //             Commands.waitUntil(shooter::atRPM).andThen(feeder.feedFuel())));

    controller.leftTrigger().toggleOnTrue(intake.intakeFuel());

    controller.start().whileTrue(intake.outtakeFuel());

    controller.rightBumper().whileTrue(shooter.pivotShooterUp());
    controller.leftBumper().whileTrue(shooter.pivotShooterDown());

    controller.a().toggleOnTrue(conveyor.transportBallsReverse());
    controller.b().toggleOnTrue(conveyor.transportBalls());
    controller.y().whileTrue(new AutoAimShooter(drive, vision, shooter, feeder));
    controller
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                new AutoAimShooter(drive, vision, shooter, feeder),
                Commands.waitSeconds(0.8).andThen(feeder.feedFuel())));
    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.parallel(
    //             shooter.dashboardShootTune(),
    //             Commands.sequence(Commands.waitSeconds(1.0), feeder.feedFuel())));

    controller.povUp().whileTrue(intake.turntoUp());
    controller.povDown().whileTrue(intake.turntoDown());
    controller.povRight().whileTrue(climber.climbUp());
    controller.povLeft().whileTrue(climber.climbDown());

    // // Lock to 0 degrees when A button is held
    // controller
    //     .a()
    //     .whileTrue(d
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.b().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0 degrees when B button is pressed
    // controller
    //     .start()
    //     .onTrue(
    // Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // controller.x().onTrue(pathfindToClosestHub(true));
    // controller.x().onTrue(pathfindToClosestHub(false));

    // // Auto aim command example
    // @SuppressWarnings("resource")
    // PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    // aimController.enableContinuousInput(-Math.PI, Math.PI);
    // controller
    //     .leftTrigger()
    //     .whileTrue(
    //         Commands.startRun(
    //             () -> {
    //               aimController.reset();
    //             },
    //             () -> {
    //               drive.run(0.0, aimController.calculate(vision.getTargetX(0).getRadians()));
    //             },
    //             drive));

    // controller.rightTrigger().whileTrue(drive.alignToCenterReef());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Command autoAimInterpolationSweep() {
    Translation2d hubCenter = FieldConstants.Hub.blueCenter.getTranslation();
    double hubY = hubCenter.getY();
    double holdTimeSec = 0.75;
    Command sweep = Commands.none();
    for (double distanceMeters : ShooterConstants.AutoAim.kDistanceMeters) {
      double sampleX = hubCenter.getX() + distanceMeters;
      sweep =
          sweep.andThen(
              Commands.sequence(
                  Commands.runOnce(
                      () -> drive.setPose(new Pose2d(sampleX, hubY, new Rotation2d())), drive),
                  Commands.deadline(
                      Commands.waitSeconds(holdTimeSec),
                      new AutoAimShooter(drive, vision, shooter, feeder))));
    }
    return sweep.withName("AutoAimInterpolationSweep");
  }

  public Command pathfindToClosestHub(boolean leftSide) {
    return Commands.defer(
        () ->
            AutoBuilder.pathfindToPose(
                drive.getClosestAprilTagOnHub(leftSide), hubPathfindConstraints, 0.0),
        Set.of(drive));
  }
}
