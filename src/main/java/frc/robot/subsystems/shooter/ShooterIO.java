package frc.robot.subsystems.shooter;

public interface ShooterIO {
  /**
   * Sets the speed of the shooter motors.
   *
   * @param speed The speed to set the shooter motors to.
   */
  public void setShooterSpeed(double speed);

  /** Sets shooter motors with direct voltage for characterization. */
  public void setShooterVoltage(double volts);

  /**
   * Sets the speed of the pivot motor.
   *
   * @param speed The speed to set the intake motor to.
   */
  public void setPivotSpeed(double speed);

  /** Sets pivot motor with direct voltage for characterization. */
  public void setPivotVoltage(double volts);

  /**
   * Sets the position of the pivot motor.
   *
   * @param position The position to set the pivot motor to.
   */
  public void setPivotPosition(double position);

  /**
   * Gets the encoder value of the pivot motor.
   *
   * @return The encoder value of the pivot motor.
   */
  public double getPivotPosition();
}
