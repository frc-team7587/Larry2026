package frc.robot.subsystems.intake.intakeFlywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Configs.IntakeConfig;

public class IntakeFlywheelIOSpark implements IntakeFlywheelIO {
  private final SparkFlex intakeLeaderMotor;
  private final SparkFlex intakeFollowerMotor;

  public IntakeFlywheelIOSpark() {
    intakeLeaderMotor =
        new SparkFlex(IntakeFlywheelConstants.Intake.kLeaderMotorID, MotorType.kBrushless);
    intakeFollowerMotor =
        new SparkFlex(IntakeFlywheelConstants.Intake.kFollowerMotorID, MotorType.kBrushless);
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
}
