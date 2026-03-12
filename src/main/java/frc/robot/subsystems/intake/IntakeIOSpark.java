package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.IntakeConfig;

public class IntakeIOSpark implements IntakeIO {
  private final SparkMax pivotLeaderMotor;
  private final SparkMax pivotFollowerMotor;
  private final SparkFlex intakeLeaderMotor;
  private final SparkFlex intakeFollowerMotor;

  private final RelativeEncoder pivotLeaderEncoder;
  private final SparkClosedLoopController pivotLeaderController;

  public IntakeIOSpark() {
    pivotLeaderMotor = new SparkMax(IntakeConstants.Pivot.kLeaderID, MotorType.kBrushless);
    pivotFollowerMotor = new SparkMax(IntakeConstants.Pivot.kFollowerID, MotorType.kBrushless);
    intakeLeaderMotor = new SparkFlex(IntakeConstants.Intake.kLeaderMotorID, MotorType.kBrushless);
    intakeFollowerMotor = new SparkFlex(IntakeConstants.Intake.kFollowerMotorID, MotorType.kBrushless);

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
    intakeLeaderMotor.configure(
        IntakeConfig.intakeMotorLeaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeFollowerMotor.configure(
        IntakeConfig.intakeMotorFollowerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeLeaderMotor.set(speed);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    intakeLeaderMotor.setVoltage(volts);
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
