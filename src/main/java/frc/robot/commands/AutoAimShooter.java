package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.ShooterFlywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterFlywheel.ShooterFlywheelConstants;
import frc.robot.subsystems.shooter.ShooterPivot.ShooterPivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

/**
 * Continuously aims the shooter pivot and wheel speed from distance-to-hub interpolation tables.
 */
public class AutoAimShooter extends Command {
  private final Drive drive;
  private final Vision vision;
  private final ShooterFlywheel shooterFlywheel;
  private final ShooterPivot shooterPivot;
  private final Feeder feeder;

  private final InterpolatingDoubleTreeMap pivotMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterRpmMap = new InterpolatingDoubleTreeMap();

  private final double minDistanceMeters;
  private final double maxDistanceMeters;

  public AutoAimShooter(
      Drive drive,
      Vision vision,
      ShooterFlywheel shooterFlywheel,
      ShooterPivot shooterPivot,
      Feeder feeder) {
    this.drive = drive;
    this.vision = vision;
    this.shooterFlywheel = shooterFlywheel;
    this.shooterPivot = shooterPivot;

    this.feeder = feeder;

    for (int i = 0; i < ShooterFlywheelConstants.AutoAim.kDistanceMeters.length; i++) {
      double distance = ShooterFlywheelConstants.AutoAim.kDistanceMeters[i];
      pivotMap.put(distance, ShooterFlywheelConstants.AutoAim.kPivotPosition[i]);
      shooterRpmMap.put(distance, ShooterFlywheelConstants.AutoAim.kShooterTargetRpm[i]);
    }

    minDistanceMeters = ShooterFlywheelConstants.AutoAim.kDistanceMeters[0];
    maxDistanceMeters =
        ShooterFlywheelConstants.AutoAim.kDistanceMeters[
            ShooterFlywheelConstants.AutoAim.kDistanceMeters.length - 1];

    addRequirements(shooterFlywheel, shooterPivot);
  }

  @Override
  public void execute() {
    OptionalDouble visionDistanceMeters = vision.getLatestHubDistanceMeters();

    Pose2d robotPose = drive.getPose();
    Translation2d hubCenter =
        AllianceFlipUtil.apply(FieldConstants.Hub.blueCenter).getTranslation();
    double odometryDistanceMeters = robotPose.getTranslation().getDistance(hubCenter);

    boolean usingVisionDistance = visionDistanceMeters.isPresent();
    double rawVisionDistanceMeters = visionDistanceMeters.orElse(Double.NaN);
    double adjustedVisionDistanceMeters =
        usingVisionDistance
            ? rawVisionDistanceMeters * ShooterFlywheelConstants.AutoAim.kVisionDistanceScale
                + ShooterFlywheelConstants.AutoAim.kVisionDistanceBiasMeters
            : Double.NaN;

    double distanceMeters =
        usingVisionDistance ? adjustedVisionDistanceMeters : odometryDistanceMeters;
    double clampedDistance = MathUtil.clamp(distanceMeters, minDistanceMeters, maxDistanceMeters);

    double pivotSetpoint = pivotMap.get(clampedDistance);

    // TODO
    // THIS IS HACKY
    // REMOVE THIS BEFORE COMP
    // double shooterTargetRpm = (shooterRpmMap.get(clampedDistance) + 600) / 3;
    double shooterTargetRpm = shooterRpmMap.get(clampedDistance);

    shooterPivot.setPivotPositionVoid(pivotSetpoint);
    shooterFlywheel.setVelocityRpm(shooterTargetRpm);

    Logger.recordOutput("Shooter/AutoAim/UsingVisionDistance", usingVisionDistance);
    Logger.recordOutput("Shooter/AutoAim/RawVisionDistanceMeters", rawVisionDistanceMeters);
    Logger.recordOutput(
        "Shooter/AutoAim/AdjustedVisionDistanceMeters", adjustedVisionDistanceMeters);
    Logger.recordOutput("Shooter/AutoAim/OdometryDistanceMeters", odometryDistanceMeters);
    Logger.recordOutput("Shooter/AutoAim/DistanceMeters", distanceMeters);
    Logger.recordOutput("Shooter/AutoAim/DistanceClampedMeters", clampedDistance);
    Logger.recordOutput("Shooter/AutoAim/PivotSetpoint", pivotSetpoint);
    Logger.recordOutput("Shooter/AutoAim/PivotEncoderPosition", shooterPivot.getPivotPosition());
    Logger.recordOutput("Shooter/AutoAim/RecommendedShooterTargetRpm", shooterTargetRpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooterPivot.holdPivotPosition();
    shooterFlywheel.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // public Command shootertuned(double shooterTargetRpm) {
  //   return Commands.parallel(
  //       shooter.setShooterSpeedWithTargetRpm(ShooterConstants.Top.kOutSpeed, shooterTargetRpm),
  //       Commands.sequence(Commands.waitSeconds(1.0), feeder.feedFuel()));
  // }
}
