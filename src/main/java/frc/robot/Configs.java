package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.intake.IntakePivot.IntakePivotConstants;
import frc.robot.subsystems.intake.intakeFlywheel.IntakeFlywheelConstants;
import frc.robot.subsystems.shooter.shooterFlywheel.ShooterFlywheelConstants;
import frc.robot.subsystems.shooter.shooterPivot.ShooterPivotConstants;

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
              IntakePivotConstants.kP,
              IntakePivotConstants.kI,
              IntakePivotConstants.kD,
              IntakePivotConstants.kFF)
          .outputRange(IntakePivotConstants.kMinOutput, IntakePivotConstants.kMaxOutput);
      pivotMotorFollowerConfig
          .apply(pivotMotorLeaderConfig)
          .follow(IntakePivotConstants.kLeaderID, true);
      intakeMotorLeaderConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(60)
          .inverted(false);
      intakeMotorFollowerConfig
          .apply(intakeMotorLeaderConfig)
          .follow(IntakeFlywheelConstants.Intake.kLeaderMotorID, true);
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
      topMotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              ShooterFlywheelConstants.Top.kP,
              ShooterFlywheelConstants.Top.kI,
              ShooterFlywheelConstants.Top.kD);
      topMotorConfig
          .encoder
          .positionConversionFactor(1.0 / ShooterFlywheelConstants.Top.kGearRatio)
          .velocityConversionFactor(1.0 / ShooterFlywheelConstants.Top.kGearRatio);

      // 24 & 8
      // topMotorConfig.encoder.uvwMeasurementPeriod(8).uvwAverageDepth(4);

      // bottomMotorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
      bottomMotorConfig
          .apply(topMotorConfig)
          .follow(ShooterFlywheelConstants.Top.kTopMotorID, true);

      pivotMotorConfig
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12.0)
          .smartCurrentLimit(40)
          .inverted(false);
      pivotMotorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(
              ShooterPivotConstants.kP,
              ShooterPivotConstants.kI,
              ShooterPivotConstants.kD,
              ShooterPivotConstants.kFF)
          .outputRange(ShooterPivotConstants.kMinOutput, ShooterPivotConstants.kMaxOutput);
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
          .smartCurrentLimit(30)
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
