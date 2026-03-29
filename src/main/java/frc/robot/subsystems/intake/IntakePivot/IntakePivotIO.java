package frc.robot.subsystems.intake.IntakePivot;

public interface IntakePivotIO {
  /**
   * Sets the speed of the pivot motors.
   *
   * @param speed The speed to set the pivot motors to.
   */
  default void setPivotSpeed(double speed) {}

  /** Sets pivot motor with direct voltage for characterization. */
  default void setPivotVoltage(double volts) {}

  /**
   * Sets the position of the pivot motors.
   *
   * @param position The position to set the pivot motors to.
   */
  default void setPivotPosition(double position) {}

  /**
   * Gets the encoder value of the pivot motors.
   *
   * @return The encoder value of the pivot motors.
   */
  default double getPivotPosition() {
    return 0.0;
  }
}
