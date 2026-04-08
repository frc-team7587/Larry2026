package frc.robot.subsystems.feeder;

public class FeederIOSim implements FeederIO {
  private double feederVelocityRpm = 0.0;

  @Override
  public void setFeederSpeed(double speed) {
    setVelocity(speed * FeederConstants.kFeederFreeSpeedRpm);
  }

  @Override
  public void setFeederVoltage(double volts) {
    feederVelocityRpm = volts / 12.0 * FeederConstants.kFeederFreeSpeedRpm;
  }

  @Override
  public void setVelocity(double rpm) {
    feederVelocityRpm = rpm;
  }

  @Override
  public double getFeederVelocityRpm() {
    return feederVelocityRpm;
  }
}
