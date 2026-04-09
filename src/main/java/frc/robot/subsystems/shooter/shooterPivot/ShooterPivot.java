package frc.robot.subsystems.shooter.ShooterPivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ShooterPivot extends SubsystemBase {
  private final ShooterPivotIO shooter;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();
  private final SysIdRoutine pivotSysId;

  public ShooterPivot(ShooterPivotIO shooterPivotIO) {
    this.shooter = shooterPivotIO;
    pivotSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/PivotSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> shooter.setPivotVoltage(voltage.in(Volts)), null, this));

    // shooter.setPivotEncoderPosition(ShooterPivotConstants.kBottomPosition);
  }

  public Command setPivotPositionCom(double position) {
    return runOnce(() -> shooter.setPivotPosition(position));
  }

  public void setPivotPositionVoid(double position) {
    shooter.setPivotPosition(position);
  }

  public void holdPivotPosition() {
    shooter.setPivotPosition(shooter.getPivotPosition());
  }

  public double getPivotPosition() {
    return shooter.getPivotPosition();
  }

  public Command pivotShooterUp() {
    return startEnd(
        () -> shooter.setPivotSpeed(ShooterPivotConstants.kPivotSpeedUp),
        () -> shooter.setPivotPosition(shooter.getPivotPosition()));
  }

  public Command pivotShooterDown() {
    return startEnd(
        () -> shooter.setPivotSpeed(ShooterPivotConstants.kPivotSpeedDown),
        () -> shooter.setPivotPosition(shooter.getPivotPosition()));
  }

  @Override
  public void periodic() {
    shooter.updateInputs(inputs);
    Logger.processInputs("Shooter/Pivot", inputs);
    double pivotPosition = inputs.position;
    Logger.recordOutput("Shooter/PivotEncoderPosition", pivotPosition);
    Logger.recordOutput("Shooter/PivotPosition", pivotPosition);
    SmartDashboard.putNumber("Shooter/PivotEncoderPosition", pivotPosition);
  }

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysId.quasistatic(direction);
  }

  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysId.dynamic(direction);
  }
}
