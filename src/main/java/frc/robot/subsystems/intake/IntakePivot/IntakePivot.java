package frc.robot.subsystems.intake.IntakePivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakePivot extends SubsystemBase {
  private final IntakePivotIO pivot;
  private final SysIdRoutine pivotSysId;

  private final LoggedMechanism2d mechPanel;
  private final LoggedMechanismRoot2d mechRoot;
  private final LoggedMechanismLigament2d mechArm;
  private final LoggedMechanismRoot2d mechIntakeRoot;
  private final LoggedMechanismLigament2d mechIntake;

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

    mechPanel =
        new LoggedMechanism2d(
            Units.inchesToMeters(100),
            Units.inchesToMeters(100),
            new Color8Bit(Color.kGray)); // view panel size
    mechRoot = mechPanel.getRoot("root", Units.inchesToMeters(50), Units.inchesToMeters(0));
    mechArm =
        mechRoot.append(
            new LoggedMechanismLigament2d("arm", Units.inchesToMeters(8), 90, 10, new Color8Bit()));
    mechIntakeRoot =
        mechPanel.getRoot("intakeroot", Units.inchesToMeters(50), Units.inchesToMeters(58));
    mechIntake =
        mechArm.append(
            new LoggedMechanismLigament2d(
                "intake", Units.inchesToMeters(24), 0, 10, new Color8Bit(Color.kRed)));
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

  @Override
  public void periodic() {
    Logger.recordOutput("intakeMech", mechPanel);
    Logger.recordOutput("Intake/PivotPosition", pivot.getPivotPosition());
    // turns encoder position to degrees
    mechIntake.setAngle(new Rotation2d(pivot.getPivotPosition() - 90.0));
  }
}
