package frc.robot.subsystems.shooter.shooterPivot;

public interface ShooterPivotIO {
  /**
   * Sets the speed of the pivot motor.
   *
   * @param speed The speed to set the intake motor to.
   */
  default void setPivotSpeed(double speed) {}

  /** Sets pivot motor with direct voltage for characterization. */
  default void setPivotVoltage(double volts) {}

  /**
   * Sets the position of the pivot motor.
   *
   * @param position The position to set the pivot motor to.
   */
  default void setPivotPosition(double position) {}

  /** Sets the pivot encoder to a known position. */
  default void setPivotEncoderPosition(double position) {}

  /**
   * Gets the encoder value of the pivot motor.
   *
   * @return The encoder value of the pivot motor.
   */
  default double getPivotPosition() {
    return 0.0;
  }
}
