package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Configs.FeederConfig;

public class FeederIOSpark implements FeederIO {
  private final SparkMax feederMotor;
  private final RelativeEncoder feederEncoder;
  private final SparkClosedLoopController feederController;
  private final SimpleMotorFeedforward feedforwardController;

  public FeederIOSpark() {
    feederMotor = new SparkMax(FeederConstants.kMotorID, MotorType.kBrushless);
    feederEncoder = feederMotor.getEncoder();
    feederController = feederMotor.getClosedLoopController();
    feedforwardController = new SimpleMotorFeedforward(FeederConstants.kS, FeederConstants.kV);

    feederMotor.configure(
        FeederConfig.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.connected = feederMotor.getLastError() == REVLibError.kOk;
    inputs.velocityRpm = feederEncoder.getVelocity();
    inputs.appliedVolts = feederMotor.getAppliedOutput() * feederMotor.getBusVoltage();
    inputs.currentAmps = feederMotor.getOutputCurrent();
  }

  @Override
  public void setFeederSpeed(double speed) {
    setVelocity(speed);
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double rpm) {
    feederController.setSetpoint(
        rpm,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforwardController.calculate(rpm),
        ArbFFUnits.kVoltage);
  }

  @Override
  public double getFeederVelocityRpm() {
    return feederEncoder.getVelocity();
  }

  @Override
  public void stop() {
    feederMotor.stopMotor();
  }
}
