package frc.robot.subsystems.feeder;

public interface FeederIO {
  /**
   * Sets the feeder target speed as a normalized output.
   *
   * @param speed The normalized speed to set the feeder motor to.
   */
  default void setFeederSpeed(double speed) {}

  /** Sets feeder motor with direct voltage for characterization. */
  default void setFeederVoltage(double volts) {}

  /** Sets the feeder target velocity in mechanism rpm. */
  default void setVelocity(double rpm) {}

  /** Gets the feeder velocity in mechanism rpm. */
  default double getFeederVelocityRpm() {
    return 0.0;
  }
}
