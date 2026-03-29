package frc.robot.subsystems.conveyor;

public interface ConveyorIO {
  /**
   * Sets the speed of the conveyor motor.
   *
   * @param speed The speed to set the conveyor motor to.
   */
  default void setConveyorSpeed(double speed) {}

  /** Sets conveyor motor with direct voltage for characterization. */
  default void setConveyorVoltage(double volts) {}
}
