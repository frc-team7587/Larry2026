package frc.robot.subsystems.IntakePivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO pivot;
  private final SysIdRoutine pivotSysId;

  public IntakePivot(IntakePivotIO pivot) {
    this.pivot = pivot;
    pivotSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/PivotSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> pivot.setPivotVoltage(voltage.in(Volts)), null, this));
  }

  public Command setPivotPosition(double position) {
    return run(() -> pivot.setPivotPosition(position));
  }

  public Command turntoUp() {
    return startEnd(
        () -> pivot.setPivotSpeed(IntakePivotConstants.kPivotSpeedUp),
        () -> pivot.setPivotPosition(pivot.getPivotPosition())); // this really has to be fixed
  }

  public Command turntoDown() {
    return startEnd(
        () -> pivot.setPivotSpeed(IntakePivotConstants.kPivotSpeedDown),
        () -> pivot.setPivotPosition(pivot.getPivotPosition())); // this really has to be fixed
  }

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysId.quasistatic(direction);
  }

  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysId.dynamic(direction);
  }
}
