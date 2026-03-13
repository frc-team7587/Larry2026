package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MovingShotAiming;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

/**
 * Continuously aims the shooter pivot and wheel speed from distance-to-hub interpolation tables.
 */
public class AutoAimShooter extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;
  private final Feeder feeder;

  private final InterpolatingDoubleTreeMap pivotMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterRpmMap = new InterpolatingDoubleTreeMap();

  private final double minDistanceMeters;
  private final double maxDistanceMeters;

  public AutoAimShooter(Drive drive, Vision vision, Shooter shooter, Feeder feeder) {
    this.drive = drive;
    this.vision = vision;
    this.shooter = shooter;
    this.feeder = feeder;

    for (int i = 0; i < ShooterConstants.AutoAim.kDistanceMeters.length; i++) {
      double distance = ShooterConstants.AutoAim.kDistanceMeters[i];
      pivotMap.put(distance, ShooterConstants.AutoAim.kPivotPosition[i]);
      shooterRpmMap.put(distance, ShooterConstants.AutoAim.kShooterTargetRpm[i]);
    }

    minDistanceMeters = ShooterConstants.AutoAim.kDistanceMeters[0];
    maxDistanceMeters =
        ShooterConstants.AutoAim.kDistanceMeters[
            ShooterConstants.AutoAim.kDistanceMeters.length - 1];

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    OptionalDouble visionDistanceMeters = vision.getLatestHubDistanceMeters();

    Pose2d robotPose = drive.getPose();
    Translation2d hubCenter =
        AllianceFlipUtil.apply(FieldConstants.Hub.blueCenter).getTranslation();
    double odometryDistanceMeters = robotPose.getTranslation().getDistance(hubCenter);
    var fieldRelativeSpeeds = drive.getFieldRelativeChassisSpeeds();
    Translation2d fieldRelativeVelocity =
        new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    MovingShotAiming.AimSolution aimSolution =
        MovingShotAiming.solve(robotPose, fieldRelativeSpeeds, fieldRelativeVelocity, hubCenter);
    double predictedDistanceMeters = aimSolution.predictedDistanceMeters();
    double distanceLeadMeters = predictedDistanceMeters - odometryDistanceMeters;
    double translationSpeedMetersPerSec = fieldRelativeVelocity.getNorm();

    boolean usingVisionDistance = visionDistanceMeters.isPresent();
    double rawVisionDistanceMeters = visionDistanceMeters.orElse(Double.NaN);
    double adjustedVisionDistanceMeters =
        usingVisionDistance
            ? rawVisionDistanceMeters * ShooterConstants.AutoAim.kVisionDistanceScale
                + ShooterConstants.AutoAim.kVisionDistanceBiasMeters
            : Double.NaN;

    boolean blendVisionDistance =
        usingVisionDistance
            && translationSpeedMetersPerSec <= ShooterConstants.AutoAim.kVisionMaxBlendSpeedMetersPerSec;
    double baseDistanceMeters = blendVisionDistance ? adjustedVisionDistanceMeters : odometryDistanceMeters;
    double distanceMeters = baseDistanceMeters + distanceLeadMeters;
    double clampedDistance = MathUtil.clamp(distanceMeters, minDistanceMeters, maxDistanceMeters);

    double pivotSetpoint = pivotMap.get(clampedDistance);

    double shooterTargetRpm = shooterRpmMap.get(clampedDistance);

    shooter.setPivotPosition(pivotSetpoint);
    // shootertuned(shooterTargetRpm);

    double clampedRpm =
        MathUtil.clamp(
            shooterTargetRpm,
            -ShooterConstants.Control.kDashboardMaxTargetRpm,
            ShooterConstants.Control.kDashboardMaxTargetRpm);
    double mappedOutput = clampedRpm / ShooterConstants.Control.kDashboardMaxTargetRpm;

    shooter.setShooterSpeedWithTargetRpm(mappedOutput, shooterTargetRpm);

    Logger.recordOutput("Shooter/AutoAim/UsingVisionDistance", usingVisionDistance);
    Logger.recordOutput("Shooter/AutoAim/RawVisionDistanceMeters", rawVisionDistanceMeters);
    Logger.recordOutput(
        "Shooter/AutoAim/AdjustedVisionDistanceMeters", adjustedVisionDistanceMeters);
    Logger.recordOutput("Shooter/AutoAim/OdometryDistanceMeters", odometryDistanceMeters);
    Logger.recordOutput("Shooter/AutoAim/PredictedDistanceMeters", predictedDistanceMeters);
    Logger.recordOutput("Shooter/AutoAim/DistanceLeadMeters", distanceLeadMeters);
    Logger.recordOutput("Shooter/AutoAim/TranslationSpeedMetersPerSec", translationSpeedMetersPerSec);
    Logger.recordOutput("Shooter/AutoAim/VisionDistanceBlended", blendVisionDistance);
    Logger.recordOutput("Shooter/AutoAim/LookaheadSec", aimSolution.lookaheadSeconds());
    Logger.recordOutput("Shooter/AutoAim/DistanceMeters", distanceMeters);
    Logger.recordOutput("Shooter/AutoAim/DistanceClampedMeters", clampedDistance);
    Logger.recordOutput("Shooter/AutoAim/PivotSetpoint", pivotSetpoint);
    Logger.recordOutput("Shooter/AutoAim/PivotEncoderPosition", shooter.getPivotPosition());
    Logger.recordOutput("Shooter/AutoAim/RecommendedShooterTargetRpm", shooterTargetRpm);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.holdPivotPosition();
    shooter.stopShooter();
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
