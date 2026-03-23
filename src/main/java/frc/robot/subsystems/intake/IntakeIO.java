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
}
