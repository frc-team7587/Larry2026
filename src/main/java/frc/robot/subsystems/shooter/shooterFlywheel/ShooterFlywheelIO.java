package frc.robot.subsystems.shooter.ShooterFlywheel;

public interface ShooterFlywheelIO {
  /**
   * Sets the speed of the shooter motors.
   *
   * @param speed The speed to set the shooter motors to.
   */
  default void setShooterSpeed(double speed) {}

  /** Sets shooter motors with direct voltage for characterization. */
  default void setShooterVoltage(double volts) {}

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
