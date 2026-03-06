package frc.robot.subsystems.climber;

public interface ClimberIO {
  /**
   * Sets the speed of the climber motor.
   *
   * @param speed The speed to set the climber motor to.
   */
  public void setClimberSpeed(double speed);

  /** Sets climber motor with direct voltage for characterization. */
  public void setClimberVoltage(double volts);

  /**
   * Sets target climber position in output rotations.
   *
   * @param outputRotations Target output rotations.
   */
  public void setClimberPosition(double outputRotations);

  /**
   * Gets climber position in output rotations.
   *
   * @return Current output rotations.
   */
  public double getClimberPosition();
}
