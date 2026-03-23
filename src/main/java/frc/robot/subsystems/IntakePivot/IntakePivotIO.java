package frc.robot.subsystems.IntakePivot;

public interface IntakePivotIO {
  /**
   * Sets the speed of the pivot motors.
   *
   * @param speed The speed to set the pivot motors to.
   */
  public void setPivotSpeed(double speed);

  /** Sets pivot motor with direct voltage for characterization. */
  public void setPivotVoltage(double volts);

  /**
   * Sets the position of the pivot motors.
   *
   * @param position The position to set the pivot motors to.
   */
  public void setPivotPosition(double position);

  /**
   * Gets the encoder value of the pivot motors.
   *
   * @return The encoder value of the pivot motors.
   */
  public double getPivotPosition();
}
