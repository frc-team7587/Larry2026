package frc.robot.subsystems.shooter.ShooterFlywheel;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFlywheelIO {
  @AutoLog
  public static class ShooterFlywheelIOInputs {
    public boolean topConnected = false;
    public boolean bottomConnected = false;
    public double velocityRpm = 0.0;
    public double topAppliedVolts = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double topCurrentAmps = 0.0;
    public double bottomCurrentAmps = 0.0;

    public double bottomRpm = 0.0;
  }

  default void updateInputs(ShooterFlywheelIOInputs inputs) {}

  /**
   * Sets the speed of the shooter motors.
   *
   * @param speed The speed to set the shooter motors to.
   */
  default void setShooterSpeed(double speed) {}

  /** Sets shooter motors with direct voltage for characterization. */
  default void setShooterVoltage(double volts) {}

  default void setDuelVoltage(double top_volts, double bot_volts) {}

  /**
   * Gets the shooter wheel velocity in rpm.
   *
   * @return The shooter wheel velocity in rpm.
   */
  default double getShooterVelocityRpm() {
    return 0.0;
  }

  default void setVelocity(double rpm) {}
}
