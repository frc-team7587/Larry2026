package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;

public final class MovingShotAiming {
  private MovingShotAiming() {}

  public static AimSolution solve(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeVelocity,
      Translation2d desiredFieldRelativeVelocity,
      Translation2d targetTranslation) {
    Translation2d measuredVelocity =
        new Translation2d(fieldRelativeVelocity.vxMetersPerSecond, fieldRelativeVelocity.vyMetersPerSecond);
    Translation2d predictionVelocity = measuredVelocity.plus(desiredFieldRelativeVelocity).times(0.5);

    double lookaheadSeconds = computeLookahead(robotPose.getTranslation(), predictionVelocity, targetTranslation);
    Translation2d predictedReleasePosition =
        robotPose.getTranslation().plus(predictionVelocity.times(lookaheadSeconds));
    Rotation2d targetHeading = targetTranslation.minus(predictedReleasePosition).getAngle();

    double sampleLookahead =
        Math.min(
            ShooterConstants.AutoAim.Potential.kMaxLookaheadSec,
            lookaheadSeconds + ShooterConstants.AutoAim.Potential.kHeadingRateSampleSec);
    Translation2d sampledReleasePosition =
        robotPose.getTranslation().plus(predictionVelocity.times(sampleLookahead));
    Rotation2d sampledTargetHeading = targetTranslation.minus(sampledReleasePosition).getAngle();

    double headingRateRadPerSec =
        sampledTargetHeading.minus(targetHeading).getRadians()
            / ShooterConstants.AutoAim.Potential.kHeadingRateSampleSec;
    double headingErrorRad = targetHeading.minus(robotPose.getRotation()).getRadians();

    double desiredSpeed = desiredFieldRelativeVelocity.getNorm();
    double speedPotential =
        MathUtil.clamp(
            desiredSpeed / ShooterConstants.AutoAim.Potential.kVelocityForFullReductionMetersPerSec,
            0.0,
            1.0);
    double headingPotential =
        MathUtil.clamp(
            Math.abs(headingErrorRad)
                / ShooterConstants.AutoAim.Potential.kFullReductionHeadingErrorRad,
            0.0,
            1.0);
    double translationScale =
        MathUtil.interpolate(
            1.0,
            ShooterConstants.AutoAim.Potential.kMinTranslationScale,
            headingPotential * headingPotential * speedPotential);

    return new AimSolution(
        targetHeading,
        headingRateRadPerSec,
        translationScale,
        predictedReleasePosition.getDistance(targetTranslation),
        lookaheadSeconds,
        predictedReleasePosition,
        headingErrorRad);
  }

  private static double computeLookahead(
      Translation2d robotTranslation,
      Translation2d fieldRelativeVelocity,
      Translation2d targetTranslation) {
    double lookaheadSeconds = ShooterConstants.AutoAim.Potential.kBaseLookaheadSec;
    for (int i = 0; i < 3; i++) {
      Translation2d predictedTranslation =
          robotTranslation.plus(fieldRelativeVelocity.times(lookaheadSeconds));
      double predictedDistanceMeters = predictedTranslation.getDistance(targetTranslation);
      lookaheadSeconds =
          ShooterConstants.AutoAim.Potential.kBaseLookaheadSec
              + predictedDistanceMeters
                  * ShooterConstants.AutoAim.Potential.kDistanceLookaheadPerMeterSec
              + fieldRelativeVelocity.getNorm()
                  * ShooterConstants.AutoAim.Potential.kVelocityLookaheadPerMeterPerSec;
      lookaheadSeconds =
          MathUtil.clamp(
              lookaheadSeconds,
              ShooterConstants.AutoAim.Potential.kMinLookaheadSec,
              ShooterConstants.AutoAim.Potential.kMaxLookaheadSec);
    }
    return lookaheadSeconds;
  }

  public record AimSolution(
      Rotation2d targetHeading,
      double targetHeadingRateRadPerSec,
      double translationScale,
      double predictedDistanceMeters,
      double lookaheadSeconds,
      Translation2d predictedReleasePosition,
      double headingErrorRad) {}
}
