package frc.robot.subsystems.intake.intakeFlywheel;

public interface IntakeFlywheelIO {
  /**
   * Sets the speed of the intake motor.
   *
   * @param speed The speed to set the intake motor to.
   */
  default void setIntakeSpeed(double speed) {}

  /** Sets intake motor with direct voltage for characterization. */
  default void setIntakeVoltage(double volts) {}
}
