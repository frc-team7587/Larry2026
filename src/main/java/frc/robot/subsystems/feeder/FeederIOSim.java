package frc.robot.subsystems.feeder;

public class FeederIOSim implements FeederIO {
  private double feederVelocityRpm = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.connected = true;
    inputs.velocityRpm = feederVelocityRpm;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setFeederSpeed(double speed) {
    setVelocity(speed);
  }

  @Override
  public void setFeederVoltage(double volts) {
    appliedVolts = volts;
    feederVelocityRpm = volts / 12.0;
  }

  @Override
  public void setVelocity(double rpm) {
    appliedVolts = 0.0;
    feederVelocityRpm = rpm;
  }

  @Override
  public double getFeederVelocityRpm() {
    return feederVelocityRpm;
  }
}
