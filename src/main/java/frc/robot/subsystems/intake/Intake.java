package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intake;
  private final SysIdRoutine intakeSysId;
  private final SysIdRoutine pivotSysId;

  public Intake(IntakeIO intake) {
    this.intake = intake;
    intakeSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/RollerSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> intake.setIntakeVoltage(voltage.in(Volts)), null, this));
    pivotSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/PivotSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> intake.setPivotVoltage(voltage.in(Volts)), null, this));
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
        () -> intake.setPivotPosition(intake.getPivotPosition()));
  }

  public Command turntoDown() {
    return startEnd(
        () -> intake.setPivotSpeed(IntakeConstants.Pivot.kPivotSpeedDown),
        () -> intake.setPivotPosition(intake.getPivotPosition()));
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intakeSysId.quasistatic(direction);
  }

  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeSysId.dynamic(direction);
  }

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysId.quasistatic(direction);
  }

  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysId.dynamic(direction);
  }
}
