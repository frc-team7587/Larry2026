package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ShooterConfig;

public class ShooterIOSpark implements ShooterIO {
  private final SparkMax topMotor;
  private final SparkMax bottomMotor;
  private final SparkMax pivotMotor;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotController;

  public ShooterIOSpark() {
    topMotor = new SparkMax(ShooterConstants.Top.kTopMotorID, MotorType.kBrushless);
    bottomMotor = new SparkMax(ShooterConstants.Bottom.kBottomMotorID, MotorType.kBrushless);
    pivotMotor = new SparkMax(ShooterConstants.Pivot.kPivotMotorID, MotorType.kBrushless);

    topEncoder = topMotor.getEncoder();
    pivotEncoder = pivotMotor.getEncoder();
    pivotController = pivotMotor.getClosedLoopController();

    topMotor.configure(
        ShooterConfig.topMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    bottomMotor.configure(
        ShooterConfig.bottomMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotMotor.configure(
        ShooterConfig.pivotMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setShooterSpeed(double speed) {
    topMotor.set(speed);
  }

  @Override
  public void setShooterVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public double getShooterVelocityRpm() {
    return topEncoder.getVelocity();
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setVoltage(volts);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotController.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }
}
