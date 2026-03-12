package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Configs {
  public static final class IntakeConfig {
    public static final SparkMaxConfig pivotMotorLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotMotorFollowerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeMotorLeaderConfig = new SparkMaxConfig();
    public static final SparkMaxConfig intakeMotorFollowerConfig = new SparkMaxConfig();
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
          .pidf(
              IntakeConstants.Pivot.kP,
              IntakeConstants.Pivot.kI,
              IntakeConstants.Pivot.kD,
              IntakeConstants.Pivot.kFF)
          .outputRange(IntakeConstants.Pivot.kMinOutput, IntakeConstants.Pivot.kMaxOutput);
      pivotMotorFollowerConfig
          .apply(pivotMotorLeaderConfig)
          .follow(IntakeConstants.Pivot.kLeaderID, true);
      intakeMotorLeaderConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      intakeMotorFollowerConfig
          .apply(intakeMotorLeaderConfig)
          .follow(IntakeConstants.Intake.kLeaderMotorID, true);
    }
  }

  public static final class ShooterConfig {
    public static final SparkMaxConfig topMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig bottomMotorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    public static final SoftLimitConfig shooterSoftLimit = new SoftLimitConfig();

    static {
      topMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      bottomMotorConfig.apply(topMotorConfig).follow(ShooterConstants.Top.kTopMotorID, true);
      pivotMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      pivotMotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(
              ShooterConstants.Pivot.kP,
              ShooterConstants.Pivot.kI,
              ShooterConstants.Pivot.kD,
              ShooterConstants.Pivot.kFF)
          .outputRange(ShooterConstants.Pivot.kMinOutput, ShooterConstants.Pivot.kMaxOutput);
    }
  }

  public static final class FeederConfig {
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    static {
      motorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
    }
  }

  public static final class ConveyorConfig {
    public static final SparkMaxConfig conveyorMotorConfig = new SparkMaxConfig();

    static {
      conveyorMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
    }
  }

  public static final class ClimberConfig {
    public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

    static {
      motorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      motorConfig
          .encoder
          .positionConversionFactor(1.0 / ClimberConstants.kGearRatio)
          .velocityConversionFactor(1.0 / ClimberConstants.kGearRatio);
      motorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, ClimberConstants.kFF)
          .outputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);
    }
  }
}
