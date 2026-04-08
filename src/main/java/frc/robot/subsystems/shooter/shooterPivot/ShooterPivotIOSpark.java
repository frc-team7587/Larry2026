package frc.robot.subsystems.shooter.shooterPivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ShooterConfig;
import java.util.List;

public class ShooterPivotIOSpark implements ShooterPivotIO {
  private final SparkMax pivotMotor;

  private final RelativeEncoder pivotEncoder;

  private final SparkClosedLoopController pivotController;

  public ShooterPivotIOSpark(List<SparkBase> motors) {
    pivotMotor = new SparkMax(ShooterPivotConstants.kPivotMotorID, MotorType.kBrushless);
    motors.add(pivotMotor);
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
