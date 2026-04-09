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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAimShooter;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.ConveyorIO;
import frc.robot.subsystems.conveyor.ConveyorIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOSpark;
import frc.robot.subsystems.intake.IntakePivot.IntakePivot;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotIO;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotIOSpark;
import frc.robot.subsystems.intake.intakeFlywheel.IntakeFlywheel;
import frc.robot.subsystems.intake.intakeFlywheel.IntakeFlywheelIO;
import frc.robot.subsystems.intake.intakeFlywheel.IntakeFlywheelIOSim;
import frc.robot.subsystems.intake.intakeFlywheel.IntakeFlywheelIOSpark;
import frc.robot.subsystems.marquee.MarqueeMessage;
import frc.robot.subsystems.marquee.MarqueeMessageBuilder;
import frc.robot.subsystems.marquee.MarqueeSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterFlywheel.ShooterFlywheelConstants;
import frc.robot.subsystems.shooter.ShooterFlywheel.ShooterFlywheelIO;
import frc.robot.subsystems.shooter.ShooterFlywheel.ShooterFlywheelIOSim;
import frc.robot.subsystems.shooter.ShooterFlywheel.ShooterFlywheelIOSpark;
import frc.robot.subsystems.shooter.ShooterPivot.ShooterPivot;
import frc.robot.subsystems.shooter.ShooterPivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.ShooterPivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooter.ShooterPivot.ShooterPivotIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static final int kDisplayTimeMS = 10000;
  private static final List<MarqueeMessage> kMessagesToDisplay = createMarqueeMessages();
  private static final double driverTranslationTriggerScale = 0.75;
  private static final double driverTurnScale = 0.5;
  private static final double controllerDeadband = 0.05;

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final IntakeFlywheel intake;
  private final IntakePivot intakePivot;
  private final ShooterFlywheel shooterFlywheel;
  private final ShooterPivot shooterPivot;
  private final Feeder feeder;
  private final Conveyor conveyor;

  // The climber is no longer installed so
  // private final Climber climber = new Climber(new ClimberIOSpark());
  private final MarqueeSubsystem marquee = MarqueeSubsystem.usbConnection(kMessagesToDisplay, 20);

  // Input devices
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private static final String shooterDashboardTargetRpmKey = "Shooter/DashboardTargetRpm";
  private static final String shooterDashboardOutputKey = "Shooter/DashboardMappedOutput";
  private static final double driverSlowModeScale = 0.35;

  private static final PathConstraints hubPathfindConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  private record RobotSubsystems(
      Drive drive,
      Vision vision,
      IntakeFlywheel intake,
      IntakePivot intakePivot,
      ShooterFlywheel shooterFlywheel,
      ShooterPivot shooterPivot,
      Feeder feeder,
      Conveyor conveyor) {}

  private static MarqueeMessageBuilder messageBuilder(String message, int displayTimeMs) {
    return new MarqueeMessageBuilder(message, displayTimeMs).setDelay1(40);
  }

  private static List<MarqueeMessage> createMarqueeMessages() {
    return List.of(
        messageBuilder("Thank you, Sponsors!", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build(),
        messageBuilder("The Cook Family", kDisplayTimeMS).setForegroundBlue(63).build(),
        messageBuilder("Metuchen Elks Lodge", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build(),
        messageBuilder("ADP", kDisplayTimeMS).setForegroundRed(63).build(),
        messageBuilder("The Cottell Family", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build(),
        messageBuilder("Picatinny Stem", kDisplayTimeMS).setForegroundRed(63).build(),
        messageBuilder("Whole Foods", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundRed(31)
            .build(),
        messageBuilder("The Mintz Family", kDisplayTimeMS).setForegroundBlue(63).build(),
        messageBuilder("Geico", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build(),
        messageBuilder("Chipotle", kDisplayTimeMS)
            .setForegroundBlue(21)
            .setForegroundGreen(21)
            .setForegroundRed(21)
            .build(),
        messageBuilder("Bagel Pantry", kDisplayTimeMS).setForegroundGreen(63).build(),
        messageBuilder("The McGrory Family", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build(),
        messageBuilder("Metuchen Diner", kDisplayTimeMS).setForegroundRed(63).build(),
        messageBuilder("Latin Port", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundRed(31)
            .build(),
        messageBuilder("The Vohra Family", kDisplayTimeMS).setForegroundBlue(63).build(),
        messageBuilder("L&L Pizza and Pasta", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build(),
        messageBuilder("Nagy Automotive", kDisplayTimeMS).setForegroundGreen(63).build(),
        messageBuilder("Small Quantities New Jersey", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build(),
        messageBuilder("Bonny's BBQ", kDisplayTimeMS).setForegroundRed(63).build(),
        messageBuilder("The Sinclair Family", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundRed(31)
            .build(),
        messageBuilder("Antonio's Brick Oven Pizza", kDisplayTimeMS).setForegroundBlue(63).build(),
        messageBuilder("Manasquan Bank", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build(),
        messageBuilder("Jersey Mike's", 0).setForegroundGreen(31).setForegroundRed(31).build());
  }

  private static Vision createVision(Drive drive) {
    if (!Constants.enableVision) {
      return new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }

    switch (Constants.currentMode) {
      case REAL:
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(VisionConstants.cameraNames[0], drive::getRotation),
            new VisionIOLimelight(VisionConstants.cameraNames[1], drive::getRotation));
      case SIM:
        return new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.cameraNames[0], VisionConstants.robotToCameras[0], drive::getPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.cameraNames[1], VisionConstants.robotToCameras[1], drive::getPose));
      default:
        return new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }
  }

  private static RobotSubsystems createSubsystems() {
    switch (Constants.currentMode) {
      case REAL:
        Drive realDrive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        return new RobotSubsystems(
            realDrive,
            createVision(realDrive),
            new IntakeFlywheel(new IntakeFlywheelIOSpark()),
            new IntakePivot(new IntakePivotIOSpark()),
            new ShooterFlywheel(new ShooterFlywheelIOSpark()),
            new ShooterPivot(new ShooterPivotIOSpark()),
            new Feeder(new FeederIOSpark()),
            new Conveyor(new ConveyorIOSpark()));

      case SIM:
        Drive simDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        return new RobotSubsystems(
            simDrive,
            createVision(simDrive),
            new IntakeFlywheel(new IntakeFlywheelIOSim()),
            new IntakePivot(new IntakePivotIOSim()),
            new ShooterFlywheel(new ShooterFlywheelIOSim()),
            new ShooterPivot(new ShooterPivotIOSim()),
            new Feeder(new FeederIOSim()),
            new Conveyor(new ConveyorIO() {}));

      default:
        Drive replayDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        return new RobotSubsystems(
            replayDrive,
            createVision(replayDrive),
            new IntakeFlywheel(new IntakeFlywheelIO() {}),
            new IntakePivot(new IntakePivotIO() {}),
            new ShooterFlywheel(new ShooterFlywheelIO() {}),
            new ShooterPivot(new ShooterPivotIO() {}),
            new Feeder(new FeederIO() {}),
            new Conveyor(new ConveyorIO() {}));
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotSubsystems subsystems = createSubsystems();
    drive = subsystems.drive();
    vision = subsystems.vision();
    intake = subsystems.intake();
    intakePivot = subsystems.intakePivot();
    shooterFlywheel = subsystems.shooterFlywheel();
    shooterPivot = subsystems.shooterPivot();
    feeder = subsystems.feeder();
    conveyor = subsystems.conveyor();

    registerNamedCommands();
    autoChooser = createAutoChooser();
    configureShooterDashboard();
    configureButtonBindings();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "shoot preload", createAutoAimAndFeedCommand().withName("ShootPreload"));
    NamedCommands.registerCommand("active floor", conveyor.transportBalls());
    NamedCommands.registerCommand("intake down", intakePivot.setPivotPosition(0));
    NamedCommands.registerCommand("shooter down", shooterPivot.setPivotPositionCom(0));
    NamedCommands.registerCommand("intake fuel", intake.intakeFuel());
    NamedCommands.registerCommand("intake up", intakePivot.turntoUp());
    NamedCommands.registerCommand("intake to position", intakePivot.setPivotPosition(-4.31));
  }

  private LoggedDashboardChooser<Command> createAutoChooser() {
    LoggedDashboardChooser<Command> chooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    chooser.addOption("AutoAim Interpolation Sweep (Sim)", autoAimInterpolationSweep());
    return chooser;
  }

  private void configureShooterDashboard() {
    SmartDashboard.putNumber(
        shooterDashboardTargetRpmKey, ShooterFlywheelConstants.Control.kDashboardDefaultTargetRpm);
    SmartDashboard.putNumber(shooterDashboardOutputKey, 0.0);
    SmartDashboard.putNumber("Shooter/MeasuredVelocityRpm", 0.0);
    SmartDashboard.putNumber("Shooter/CurrentTargetVelocityRpm", 0.0);
    SmartDashboard.putNumber("Shooter/PivotEncoderPosition", 0.0);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(createDriverDriveCommand());
  }

  private void configureDriverBindings() {
    driver
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAlignToHub(
                drive, this::getDriverScaledLeftY, this::getDriverScaledLeftX));
  }

  private void configureOperatorBindings() {
    operator
        .leftTrigger()
        .toggleOnTrue(
            Commands.parallel(
                intake.intakeFuel(() -> operator.getHID().getXButton()),
                createOperatorRumbleCommand(RumbleType.kLeftRumble)));

    operator.start().whileTrue(intake.outtakeFuel());

    operator.rightBumper().whileTrue(shooterPivot.pivotShooterUp());
    operator.leftBumper().whileTrue(shooterPivot.pivotShooterDown());

    operator
        .a()
        .toggleOnTrue(
            Commands.parallel(
                conveyor.transportBallsReverse(),
                createOperatorRumbleCommand(RumbleType.kRightRumble)));

    operator.x().whileTrue(feeder.runStatic());

    operator
        .b()
        .toggleOnTrue(
            Commands.parallel(
                conveyor.transportBalls(), createOperatorRumbleCommand(RumbleType.kRightRumble)));

    operator
        .y()
        .whileTrue(new AutoAimShooter(drive, vision, shooterFlywheel, shooterPivot, feeder));

    Trigger autoAimShotTrigger = operator.rightTrigger();
    autoAimShotTrigger.whileTrue(createAutoAimAndFeedCommand());

    operator.povUp().whileTrue(intakePivot.turntoUp());
    operator.povDown().whileTrue(intakePivot.turntoDown());
  }

  private Command createDriverDriveCommand() {
    return DriveCommands.joystickDrive(
        drive, this::getDriverForwardInput, this::getDriverStrafeInput, this::getDriverTurnInput);
  }

  private Command createAutoAimAndFeedCommand() {
    return Commands.parallel(
        new AutoAimShooter(drive, vision, shooterFlywheel, shooterPivot, feeder),
        Commands.waitSeconds(0.8).andThen(feeder.feedFuel()));
  }

  private Command createOperatorRumbleCommand(RumbleType rumbleType) {
    return Commands.startEnd(
        () -> operator.setRumble(rumbleType, 1.0), () -> operator.setRumble(rumbleType, 0.0));
  }

  private double getDriverForwardInput() {
    return -MathUtil.applyDeadband(
        getDriverTranslationScale() * driver.getLeftY(), controllerDeadband);
  }

  private double getDriverStrafeInput() {
    return -MathUtil.applyDeadband(
        getDriverTranslationScale() * driver.getLeftX(), controllerDeadband);
  }

  private double getDriverTurnInput() {
    return -MathUtil.applyDeadband(driverTurnScale * driver.getRightX(), controllerDeadband);
  }

  private double getDriverTranslationScale() {
    return 1 - driverTranslationTriggerScale * driver.getRightTriggerAxis();
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
    for (double distanceMeters : ShooterFlywheelConstants.AutoAim.kDistanceMeters) {
      double sampleX = hubCenter.getX() + distanceMeters;
      sweep =
          sweep.andThen(
              Commands.sequence(
                  Commands.runOnce(
                      () -> drive.setPose(new Pose2d(sampleX, hubY, new Rotation2d())), drive),
                  Commands.deadline(
                      Commands.waitSeconds(holdTimeSec),
                      new AutoAimShooter(drive, vision, shooterFlywheel, shooterPivot, feeder))));
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

  private double getDriverScaledLeftY() {
    return -driver.getLeftY() * getDriverSlowModeScale();
  }

  private double getDriverScaledLeftX() {
    return -driver.getLeftX() * getDriverSlowModeScale();
  }

  private double getDriverSlowModeScale() {
    return driver.rightTrigger().getAsBoolean() ? driverSlowModeScale : 1.0;
  }
}
