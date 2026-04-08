package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ClimberConfig;
import java.util.List;

public class ClimberIOSpark implements ClimberIO {
  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  private final SparkClosedLoopController climberController;

  public ClimberIOSpark(List<SparkBase> motors) {
    climberMotor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberController = climberMotor.getClosedLoopController();
    motors.add(climberMotor);

    climberMotor.configure(
        ClimberConfig.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setClimberSpeed(double speed) {
    climberMotor.set(speed);
  }

  @Override
  public void setClimberVoltage(double volts) {
    climberMotor.setVoltage(volts);
  }

  @Override
  public void setClimberPosition(double outputRotations) {
    climberController.setSetpoint(outputRotations, ControlType.kPosition);
  }

  @Override
  public double getClimberPosition() {
    return climberEncoder.getPosition();
  }
}
