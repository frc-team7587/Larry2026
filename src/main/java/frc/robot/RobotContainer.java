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
import frc.robot.subsystems.shooter.shooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.shooterFlywheel.ShooterFlywheelConstants;
import frc.robot.subsystems.shooter.shooterFlywheel.ShooterFlywheelIO;
import frc.robot.subsystems.shooter.shooterFlywheel.ShooterFlywheelIOSim;
import frc.robot.subsystems.shooter.shooterFlywheel.ShooterFlywheelIOSpark;
import frc.robot.subsystems.shooter.shooterPivot.ShooterPivot;
import frc.robot.subsystems.shooter.shooterPivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.shooterPivot.ShooterPivotIOSim;
import frc.robot.subsystems.shooter.shooterPivot.ShooterPivotIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.ArrayList;
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
  /** Messages to display on the marquee */
  private static final List<MarqueeMessage> kMessagesToDisplay;

  private static MarqueeMessageBuilder messageBuilder(String message, int displayTimeMs) {
    return new MarqueeMessageBuilder(message, displayTimeMs).setDelay1(40);
  }

  /** Populates the message list. */
  static final int kDisplayTimeMS = 10000;

  static {
    kMessagesToDisplay = new ArrayList<>();
    kMessagesToDisplay.add(
        messageBuilder("Thank you, Sponsors!", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("The Cook Family", kDisplayTimeMS).setForegroundBlue(63).build());
    kMessagesToDisplay.add(
        messageBuilder("Metuchen Elks Lodge", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build());
    kMessagesToDisplay.add(messageBuilder("ADP", kDisplayTimeMS).setForegroundRed(63).build());
    kMessagesToDisplay.add(
        messageBuilder("The Cottell Family", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("Picatinny Stem", kDisplayTimeMS).setForegroundRed(63).build());
    kMessagesToDisplay.add(
        messageBuilder("Whole Foods", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundRed(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("The Mintz Family", kDisplayTimeMS).setForegroundBlue(63).build());
    kMessagesToDisplay.add(
        messageBuilder("Geico", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build());

    kMessagesToDisplay.add(
        messageBuilder("Chipotle", kDisplayTimeMS)
            .setForegroundBlue(21)
            .setForegroundGreen(21)
            .setForegroundRed(21)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("Bagel Pantry", kDisplayTimeMS).setForegroundGreen(63).build());
    kMessagesToDisplay.add(
        messageBuilder("The McGrory Family", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("Metuchen Diner", kDisplayTimeMS).setForegroundRed(63).build());
    kMessagesToDisplay.add(
        messageBuilder("Latin Port", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundRed(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("The Vohra Family", kDisplayTimeMS).setForegroundBlue(63).build());
    kMessagesToDisplay.add(
        messageBuilder("L&L Pizza and Pasta", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("Nagy Automotive", kDisplayTimeMS).setForegroundGreen(63).build());
    kMessagesToDisplay.add(
        messageBuilder("Small Quantities New Jersey", kDisplayTimeMS)
            .setForegroundGreen(31)
            .setForegroundRed(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("Bonny's BBQ", kDisplayTimeMS).setForegroundRed(63).build());
    kMessagesToDisplay.add(
        messageBuilder("The Sinclair Family", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundRed(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("Antonio's Brick Oven Pizza", kDisplayTimeMS).setForegroundBlue(63).build());
    kMessagesToDisplay.add(
        messageBuilder("Manasquan Bank", kDisplayTimeMS)
            .setForegroundBlue(31)
            .setForegroundGreen(31)
            .build());
    kMessagesToDisplay.add(
        messageBuilder("Jersey Mike's", 0).setForegroundGreen(31).setForegroundRed(31).build());
  }

  // Subsystems
  // private final Vision vision;
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
  private static final double driverTurnScale = 0.7;

  private static final PathConstraints hubPathfindConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

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
        intakePivot = new IntakePivot(new IntakePivotIOSpark());
        intake = new IntakeFlywheel(new IntakeFlywheelIOSpark());
        shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSpark());
        shooterPivot = new ShooterPivot(new ShooterPivotIOSpark());
        feeder = new Feeder(new FeederIOSpark());
        conveyor = new Conveyor(new ConveyorIOSpark());
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
        intakePivot = new IntakePivot(new IntakePivotIOSim());
        intake = new IntakeFlywheel(new IntakeFlywheelIOSim());
        shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIOSim());
        shooterPivot = new ShooterPivot(new ShooterPivotIOSim());
        feeder = new Feeder(new FeederIOSim());
        conveyor = new Conveyor(new ConveyorIO() {});
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
        intakePivot = new IntakePivot(new IntakePivotIO() {});
        intake = new IntakeFlywheel(new IntakeFlywheelIO() {});
        shooterFlywheel = new ShooterFlywheel(new ShooterFlywheelIO() {});
        shooterPivot = new ShooterPivot(new ShooterPivotIO() {});
        feeder = new Feeder(new FeederIO() {});
        conveyor = new Conveyor(new ConveyorIO() {});
        break;
    }

    NamedCommands.registerCommand(
        "shoot preload",
        Commands.parallel(
            new AutoAimShooter(drive, vision, shooterFlywheel, shooterPivot, feeder),
            Commands.waitSeconds(0.8).andThen(feeder.feedFuel())));

    NamedCommands.registerCommand("active floor", conveyor.transportBalls());
    NamedCommands.registerCommand("intake down", intakePivot.setPivotPosition(0));
    NamedCommands.registerCommand("shooter down", shooterPivot.setPivotPositionCom(0));
    NamedCommands.registerCommand("intake fuel", intake.intakeFuel());
    NamedCommands.registerCommand("intake up", intakePivot.turntoUp());
    NamedCommands.registerCommand("intake to position", intakePivot.setPivotPosition(-4.31));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    SmartDashboard.putNumber(
        shooterDashboardTargetRpmKey, ShooterFlywheelConstants.Control.kDashboardDefaultTargetRpm);
    SmartDashboard.putNumber(shooterDashboardOutputKey, 0.0);
    SmartDashboard.putNumber("Shooter/MeasuredVelocityRpm", 0.0);
    SmartDashboard.putNumber("Shooter/CurrentTargetVelocityRpm", 0.0);
    SmartDashboard.putNumber("Shooter/PivotEncoderPosition", 0.0);

    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(b
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
    //     intakePivot.pivotSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Intake Pivot SysId (Quasistatic Reverse)",
    //     intakePivot.pivotSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Intake Pivot SysId (Dynamic Forward)",
    //     intakePivot.pivotSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Intake Pivot SysId (Dynamic Reverse)",
    //     intakePivot.pivotSysIdDynamic(SysIdRoutine.Direction.kReverse));
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

    /*
     * Driver Binds
     */

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                -MathUtil.applyDeadband(
                    (1 - 0.75 * driver.getRightTriggerAxis()) * driver.getLeftY(), 0.05),
            () ->
                -MathUtil.applyDeadband(
                    (1 - 0.75 * driver.getRightTriggerAxis()) * driver.getLeftX(), 0.05),
            () -> -MathUtil.applyDeadband(driverTurnScale * driver.getRightX(), 0.05)));

    /*
     * Climber is now uninstalled.
     * driver.povUp().whileTrue(climber.climbUp());
     * driver.povDown().whileTrue(climber.climbDown());
     */

    driver
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAlignToHub(
                drive, this::getDriverScaledLeftY, this::getDriverScaledLeftX));

    // driver
    //     .rightTrigger()
    //     .whileTrue(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> -driver.getLeftY() * 0.5,
    //             () -> -driver.getLeftX() * 0.5,
    //             () -> -driver.getRightX() * 0.5));

    /*
     * Operator Binds
     */

    operator
        .leftTrigger()
        .toggleOnTrue(
            Commands.parallel(
                intake.intakeFuel(() -> operator.getHID().getXButton()),
                Commands.startEnd(
                    () -> operator.setRumble(RumbleType.kLeftRumble, 1.0),
                    () -> operator.setRumble(RumbleType.kLeftRumble, 0.0))));

    operator.start().whileTrue(intake.outtakeFuel());

    operator.rightBumper().whileTrue(shooterPivot.pivotShooterUp());
    operator.leftBumper().whileTrue(shooterPivot.pivotShooterDown());

    operator
        .a()
        .toggleOnTrue(
            Commands.parallel(
                conveyor.transportBallsReverse(),
                Commands.startEnd(
                    () -> operator.setRumble(RumbleType.kRightRumble, 1.0),
                    () -> operator.setRumble(RumbleType.kRightRumble, 0.0))));

    operator
        .b()
        .toggleOnTrue(
            Commands.parallel(
                conveyor.transportBalls(),
                Commands.startEnd(
                    () -> operator.setRumble(RumbleType.kRightRumble, 1.0),
                    () -> operator.setRumble(RumbleType.kRightRumble, 0.0))));

    operator
        .y()
        .whileTrue(new AutoAimShooter(drive, vision, shooterFlywheel, shooterPivot, feeder));

    Trigger manualHubShotTrigger = operator.x().and(operator.rightTrigger());
    Trigger autoAimShotTrigger = operator.rightTrigger();
    // turns rpm to radians per second then sends to setVelocityRobot
    // autoAimShotTrigger.whileTrue(
    //     Commands.parallel(
    //         shooter.dashboardShootTune(), Commands.waitSeconds(0.8).andThen(feeder.feedFuel())));

    autoAimShotTrigger.whileTrue(
        Commands.parallel(
            new AutoAimShooter(drive, vision, shooterFlywheel, shooterPivot, feeder),
            Commands.waitSeconds(0.8).andThen(feeder.feedFuel())));

    operator.povUp().whileTrue(intakePivot.turntoUp());
    operator.povDown().whileTrue(intakePivot.turntoDown());

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
    // controller.leftBumper().whileTrue(intakePivot.turntoDown());
    // controller.rightBumper().whileTrue(intakePivot.turntoUp());

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
    // contro ller.back().whileTrue(new AutoAimShooter(drive, vision, shooter));

    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.parallel(
    //             shooter.shootFuel(),
    //             Commands.waitUntil(shooter::atRPM).andThen(feeder.feedFuel())));

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

    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.parallel(
    //             shooter.dashboardShootTune(),
    //             Commands.sequence(Commands.waitSeconds(1.0), feeder.feedFuel())));

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

  private double getDriverScaledRightX() {
    return -driver.getRightX() * getDriverSlowModeScale();
  }

  private double getDriverSlowModeScale() {
    return driver.rightTrigger().getAsBoolean() ? driverSlowModeScale : 1.0;
  }
}
