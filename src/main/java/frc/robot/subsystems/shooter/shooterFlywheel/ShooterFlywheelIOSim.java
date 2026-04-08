package frc.robot.subsystems.shooter.ShooterFlywheel;

public class ShooterFlywheelIOSim implements ShooterFlywheelIO {
  private double shooterSpeed = 0.0;

  @Override
  public void setShooterSpeed(double speed) {
    shooterSpeed = speed;
  }

  @Override
  public void setShooterVoltage(double volts) {
    // Assuming a simple linear relationship between voltage and speed for simulation purposes
    shooterSpeed = volts / 12.0; // Normalize to a range of -1.0 to 1.0
  }

  public double getShooterSpeed() {
    return shooterSpeed;
  }
}
