package frc.robot.subsystems.intake.intakeFlywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class IntakeFlywheel extends SubsystemBase {
  private final IntakeFlywheelIO intake;
  private final SysIdRoutine intakeSysId;

  public IntakeFlywheel(IntakeFlywheelIO intake) {
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
  }

  public void setIntakeSpeed(double speed) {
    intake.setIntakeSpeed(speed);
  }

  public Command intakeFuel() {
    return startEnd(
        () -> intake.setIntakeSpeed(IntakeFlywheelConstants.Intake.kIntakeInSpeed),
        () -> intake.setIntakeSpeed(0));
  }

  public Command outtakeFuel() {
    return startEnd(
        () -> intake.setIntakeSpeed(IntakeFlywheelConstants.Intake.kIntakeOutSpeed),
        () -> intake.setIntakeSpeed(0));
  }

  public Command stopIntake() {
    return run(() -> intake.setIntakeSpeed(0));
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intakeSysId.quasistatic(direction);
  }

  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeSysId.dynamic(direction);
  }
}
