package frc.robot.subsystems.conveyor;

public interface ConveyorIO {
  /**
   * Sets the speed of the conveyor motor.
   *
   * @param speed The speed to set the conveyor motor to.
   */
  public void setConveyorSpeed(double speed);

  /** Sets conveyor motor with direct voltage for characterization. */
  public void setConveyorVoltage(double volts);
}
