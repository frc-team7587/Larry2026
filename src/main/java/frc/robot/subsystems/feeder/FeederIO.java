package frc.robot.subsystems.feeder;

public interface FeederIO {
  /**
   * Sets the speed of the feeder motor.
   *
   * @param speed The speed to set the feeder motor to.
   */
  default void setFeederSpeed(double speed) {}

  /** Sets feeder motor with direct voltage for characterization. */
  default void setFeederVoltage(double volts) {}
}
