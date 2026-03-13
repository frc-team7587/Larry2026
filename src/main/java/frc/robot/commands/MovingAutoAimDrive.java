package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.MovingShotAiming;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MovingAutoAimDrive extends Command {
  private static final double angleKp = 5.0;
  private static final double angleKd = 0.4;
  private static final double angleMaxVelocity = 8.0;
  private static final double angleMaxAcceleration = 20.0;

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<Translation2d> targetSupplier;
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          angleKp,
          0.0,
          angleKd,
          new TrapezoidProfile.Constraints(angleMaxVelocity, angleMaxAcceleration));

  private double lastHeadingErrorRad = Double.POSITIVE_INFINITY;
  private double lastOmegaRadPerSec = Double.POSITIVE_INFINITY;

  public MovingAutoAimDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> targetSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.targetSupplier = targetSupplier;

    angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    Translation2d desiredFieldVelocity =
        linearVelocity.times(drive.getMaxLinearSpeedMetersPerSec());
    MovingShotAiming.AimSolution aimSolution =
        MovingShotAiming.solve(
            drive.getPose(), drive.getFieldRelativeChassisSpeeds(), desiredFieldVelocity, targetSupplier.get());

    Translation2d scaledFieldVelocity = desiredFieldVelocity.times(aimSolution.translationScale());
    double omega =
        angleController.calculate(
                drive.getRotation().getRadians(), aimSolution.targetHeading().getRadians())
            + aimSolution.targetHeadingRateRadPerSec()
                * ShooterConstants.AutoAim.Potential.kHeadingVelocityFeedforward;
    omega = MathUtil.clamp(omega, -drive.getMaxAngularSpeedRadPerSec(), drive.getMaxAngularSpeedRadPerSec());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            scaledFieldVelocity.getX(), scaledFieldVelocity.getY(), omega);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation()));

    lastHeadingErrorRad = aimSolution.headingErrorRad();
    lastOmegaRadPerSec = drive.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond;

    Logger.recordOutput("Drive/MovingAutoAim/TargetHeadingDeg", aimSolution.targetHeading().getDegrees());
    Logger.recordOutput("Drive/MovingAutoAim/HeadingErrorRad", lastHeadingErrorRad);
    Logger.recordOutput(
        "Drive/MovingAutoAim/TargetHeadingRateRadPerSec", aimSolution.targetHeadingRateRadPerSec());
    Logger.recordOutput("Drive/MovingAutoAim/TranslationScale", aimSolution.translationScale());
    Logger.recordOutput(
        "Drive/MovingAutoAim/PredictedDistanceMeters", aimSolution.predictedDistanceMeters());
    Logger.recordOutput("Drive/MovingAutoAim/LookaheadSec", aimSolution.lookaheadSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean isAligned() {
    return Math.abs(lastHeadingErrorRad) <= ShooterConstants.AutoAim.Potential.kOnTargetHeadingToleranceRad
        && Math.abs(lastOmegaRadPerSec)
            <= ShooterConstants.AutoAim.Potential.kOnTargetOmegaToleranceRadPerSec;
  }
}
