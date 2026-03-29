package frc.robot.subsystems.climber;

public interface ClimberIO {
  /**
   * Sets the speed of the climber motor.
   *
   * @param speed The speed to set the climber motor to.
   */
  default void setClimberSpeed(double speed) {}

  /** Sets climber motor with direct voltage for characterization. */
  default void setClimberVoltage(double volts) {}

  /**
   * Sets target climber position in output rotations.
   *
   * @param outputRotations Target output rotations.
   */
  default void setClimberPosition(double outputRotations) {}

  /**
   * Gets climber position in output rotations.
   *
   * @return Current output rotations.
   */
  default double getClimberPosition() {
    return 0.0;
  }
}
