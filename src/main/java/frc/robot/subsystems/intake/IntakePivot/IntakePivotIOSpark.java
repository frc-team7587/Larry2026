package frc.robot.subsystems.intake.IntakePivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.IntakeConfig;

public class IntakePivotIOSpark implements IntakePivotIO {
  private final SparkMax pivotLeaderMotor;
  private final SparkMax pivotFollowerMotor;

  private final RelativeEncoder pivotLeaderEncoder;
  private final SparkClosedLoopController pivotLeaderController;

  public IntakePivotIOSpark() {
    pivotLeaderMotor = new SparkMax(IntakePivotConstants.kLeaderID, MotorType.kBrushless);
    pivotFollowerMotor = new SparkMax(IntakePivotConstants.kFollowerID, MotorType.kBrushless);

    pivotLeaderEncoder = pivotLeaderMotor.getEncoder();
    pivotLeaderController = pivotLeaderMotor.getClosedLoopController();

    pivotLeaderMotor.configure(
        IntakeConfig.pivotMotorLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotFollowerMotor.configure(
        IntakeConfig.pivotMotorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotLeaderMotor.set(speed);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotLeaderMotor.setVoltage(volts);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotLeaderController.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public double getPivotPosition() {
    return pivotLeaderEncoder.getPosition();
  }
}
