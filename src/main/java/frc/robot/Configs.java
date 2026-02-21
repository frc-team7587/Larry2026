package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Configs {
  public static final class IntakeConfig {
    public static final SparkMaxConfig pivotMotorLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotMotorFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    public static final SoftLimitConfig intakeSoftLimit = new SoftLimitConfig();

    static {
      pivotMotorLeaderConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      pivotMotorLeaderConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(IntakeConstants.Pivot.kP, IntakeConstants.Pivot.kI, IntakeConstants.Pivot.kD)
          .outputRange(IntakeConstants.Pivot.kMinOutput, IntakeConstants.Pivot.kMaxOutput);
      pivotMotorFollowerConfig
          .apply(pivotMotorLeaderConfig)
          .follow(IntakeConstants.Pivot.kLeaderID, true);
      intakeMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
    }
  }

  public static final class ShooterConfig {
    public static final SparkMaxConfig topMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig bottomMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig feederMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    public static final SoftLimitConfig shooterSoftLimit = new SoftLimitConfig();

    static {
      topMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      bottomMotorConfig.apply(topMotorConfig).follow(ShooterConstants.Top.kTopMotorID, true);
      feederMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      pivotMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      pivotMotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(ShooterConstants.Pivot.kP, ShooterConstants.Pivot.kI, ShooterConstants.Pivot.kD)
          .outputRange(ShooterConstants.Pivot.kMinOutput, ShooterConstants.Pivot.kMaxOutput);
    }
  }
}
