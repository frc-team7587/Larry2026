package frc.robot.subsystems.intake;

public interface IntakeIO {
  /**
   * Sets the speed of the intake motor.
   *
   * @param speed The speed to set the intake motor to.
   */
  public void setIntakeSpeed(double speed);

  /** Sets intake motor with direct voltage for characterization. */
  public void setIntakeVoltage(double volts);

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
