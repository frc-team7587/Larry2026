package frc.robot.subsystems.shooter.ShooterPivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ShooterConfig;

public class ShooterPivotIOSpark implements ShooterPivotIO {
  private final SparkMax pivotMotor;

  private final RelativeEncoder pivotEncoder;

  private final SparkClosedLoopController pivotController;

  public ShooterPivotIOSpark() {
    pivotMotor = new SparkMax(ShooterPivotConstants.kPivotMotorID, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();
    pivotController = pivotMotor.getClosedLoopController();

    pivotMotor.configure(
        ShooterConfig.pivotMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
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
    pivotController.setSetpoint(
        position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ShooterPivotConstants.kGravityFFVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setPivotEncoderPosition(double position) {
    pivotEncoder.setPosition(position);
  }

  @Override
  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }
}
