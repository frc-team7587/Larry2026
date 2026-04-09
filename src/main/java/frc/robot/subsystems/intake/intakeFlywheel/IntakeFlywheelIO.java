package frc.robot.subsystems.intake.intakeFlywheel;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeFlywheelIO {
  @AutoLog
  public static class IntakeFlywheelIOInputs {
    public boolean connected = false;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(IntakeFlywheelIOInputs inputs) {}

  /**
   * Sets the speed of the intake motor.
   *
   * @param speed The speed to set the intake motor to.
   */
  default void setIntakeSpeed(double speed) {}

  /** Sets intake motor with direct voltage for characterization. */
  default void setIntakeVoltage(double volts) {}
}
