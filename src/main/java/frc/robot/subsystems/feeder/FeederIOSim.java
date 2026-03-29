package frc.robot.subsystems.feeder;

public class FeederIOSim implements FeederIO {
  private double feederSpeed = 0.0;

  @Override
  public void setFeederSpeed(double speed) {
    feederSpeed = speed;
  }

  @Override
  public void setFeederVoltage(double volts) {
    // Assuming a simple linear relationship between voltage and speed for simulation purposes
    feederSpeed = volts / 12.0; // Normalize to a range of -1.0 to 1.0
  }

  public double getFeederSpeed() {
    return feederSpeed;
  }
}
