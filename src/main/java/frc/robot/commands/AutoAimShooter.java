package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/** Continuously aims shooter pivot and wheel speed from distance-to-hub interpolation tables. */
public class AutoAimShooter extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;

  private final InterpolatingDoubleTreeMap pivotMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterOutputMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap shooterRpmMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap intakeOutputMap = new InterpolatingDoubleTreeMap();

  private final double minDistanceMeters;
  private final double maxDistanceMeters;

  public AutoAimShooter(Drive drive, Shooter shooter, Intake intake) {
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;

    addRequirements(shooter, intake);

    for (int i = 0; i < ShooterConstants.AutoAim.kDistanceMeters.length; i++) {
      double distance = ShooterConstants.AutoAim.kDistanceMeters[i];
      pivotMap.put(distance, ShooterConstants.AutoAim.kPivotPosition[i]);
      shooterOutputMap.put(distance, ShooterConstants.AutoAim.kShooterPercentOutput[i]);
      shooterRpmMap.put(distance, ShooterConstants.AutoAim.kShooterTargetRpm[i]);
      intakeOutputMap.put(distance, ShooterConstants.AutoAim.kIntakePercentOutput[i]);
    }

    minDistanceMeters = ShooterConstants.AutoAim.kDistanceMeters[0];
    maxDistanceMeters =
        ShooterConstants.AutoAim.kDistanceMeters[
            ShooterConstants.AutoAim.kDistanceMeters.length - 1];
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    Translation2d hubCenter =
        AllianceFlipUtil.apply(FieldConstants.Hub.blueCenter).getTranslation();

    double distanceMeters = robotPose.getTranslation().getDistance(hubCenter);
    double clampedDistance = MathUtil.clamp(distanceMeters, minDistanceMeters, maxDistanceMeters);

    double pivotSetpoint = pivotMap.get(clampedDistance);
    double shooterOutput = shooterOutputMap.get(clampedDistance);
    double shooterTargetRpm = shooterRpmMap.get(clampedDistance);
    double intakeOutput = intakeOutputMap.get(clampedDistance);

    shooter.setPivotPosition(pivotSetpoint);
    shooter.setShooterSpeedWithTargetRpm(shooterOutput, shooterTargetRpm);
    intake.setIntakeSpeed(intakeOutput);

    Logger.recordOutput("Shooter/AutoAim/DistanceMeters", distanceMeters);
    Logger.recordOutput("Shooter/AutoAim/DistanceClampedMeters", clampedDistance);
    Logger.recordOutput("Shooter/AutoAim/PivotSetpoint", pivotSetpoint);
    Logger.recordOutput("Shooter/AutoAim/ShooterPercentOutput", shooterOutput);
    Logger.recordOutput("Shooter/AutoAim/ShooterTargetRpm", shooterTargetRpm);
    Logger.recordOutput("Shooter/AutoAim/IntakePercentOutput", intakeOutput);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    shooter.holdPivotPosition();
    intake.setIntakeSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
