package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.FeederConfig;

public class FeederIOSpark implements FeederIO {
  private final SparkMax feederMotor;

  public FeederIOSpark() {
    feederMotor = new SparkMax(FeederConstants.kMotorID, MotorType.kBrushless);

    feederMotor.configure(
        FeederConfig.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }
}
