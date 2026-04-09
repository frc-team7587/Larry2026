package frc.robot.subsystems.intake.IntakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  public static class IntakePivotIOInputs {
    public boolean connected = false;
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

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
