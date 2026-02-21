package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO intake;

  public Intake(IntakeIO intake) {
    this.intake = intake;
  }

  public Command intakeFuel() {
    return startEnd(
        () -> intake.setIntakeSpeed(IntakeConstants.Intake.kIntakeInSpeed),
        () -> intake.setIntakeSpeed(0));
  }

  public Command outtakeFuel() {
    return startEnd(
        () -> intake.setIntakeSpeed(IntakeConstants.Intake.kIntakeOutSpeed),
        () -> intake.setIntakeSpeed(0));
  }

  public Command stopIntake() {
    return run(() -> intake.setIntakeSpeed(0));
  }

  public Command setPivotPosition(double position) {
    return run(() -> intake.setPivotPosition(position));
  }

  public Command turntoUp() {
    return startEnd(
        () -> intake.setPivotSpeed(IntakeConstants.Pivot.kPivotSpeedUp),
        () -> intake.setPivotSpeed(0));
  }

  public Command turntoDown() {
    return startEnd(
        () -> intake.setPivotSpeed(IntakeConstants.Pivot.kPivotSpeedDown),
        () -> intake.setPivotSpeed(0));
  }
}
