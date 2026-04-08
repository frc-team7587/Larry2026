package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.FeederConfig;

public class FeederIOSpark implements FeederIO {
  private final SparkMax feederMotor;
  private final RelativeEncoder feederEncoder;
  private final SparkClosedLoopController feederController;

  public FeederIOSpark() {
    feederMotor = new SparkMax(FeederConstants.kMotorID, MotorType.kBrushless);
    feederEncoder = feederMotor.getEncoder();
    feederController = feederMotor.getClosedLoopController();

    feederMotor.configure(
        FeederConfig.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setFeederSpeed(double speed) {
    setVelocity(speed * FeederConstants.kFeederFreeSpeedRpm);
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double rpm) {
    feederController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public double getFeederVelocityRpm() {
    return feederEncoder.getVelocity();
  }
}
