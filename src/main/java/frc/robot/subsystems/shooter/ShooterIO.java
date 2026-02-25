package frc.robot.subsystems.shooter;

public interface ShooterIO {
  /**
   * Sets the speed of the shooter motors.
   *
   * @param speed The speed to set the shooter motors to.
   */
  public void setShooterSpeed(double speed);

  /**
   * Sets the speed of the feeder motor.
   *
   * @param speed The speed to set the feeder motor to.
   */
  public void setFeederSpeed(double speed);

  /**
   * Sets the speed of the pivot motor.
   *
   * @param speed The speed to set the intake motor to.
   */
  public void setPivotSpeed(double speed);

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
