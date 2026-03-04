package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooter;
  private final SysIdRoutine wheelSysId;
  private final SysIdRoutine feederSysId;
  private final SysIdRoutine pivotSysId;

  public Shooter(ShooterIO shooter) {
    this.shooter = shooter;
    wheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("Shooter/WheelSysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> shooter.setShooterVoltage(voltage.in(Volts)), null, this));
    feederSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/FeederSysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> shooter.setFeederVoltage(voltage.in(Volts)), null, this));
    pivotSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("Shooter/PivotSysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> shooter.setPivotVoltage(voltage.in(Volts)), null, this));
  }

  public Command shootFuel() {
    return Commands.startEnd(
        () -> shooter.setShooterSpeed(ShooterConstants.Top.kOutSpeed),
        () -> shooter.setShooterSpeed(0));
  }

  public Command feedFuel() {
    return Commands.startEnd(
        () -> shooter.setFeederSpeed(ShooterConstants.Feeder.kOutSpeed),
        () -> shooter.setFeederSpeed(0));
  }

  public Command shootAndFeedFuel() {
    return Commands.startEnd(
        () -> {
          shooter.setShooterSpeed(ShooterConstants.Top.kOutSpeed);
          shooter.setFeederSpeed(ShooterConstants.Feeder.kOutSpeed);
        },
        () -> {
          shooter.setShooterSpeed(0);
          shooter.setFeederSpeed(0);
        });
  }

  public Command shootAndFeedFuelReverse() {
    return Commands.startEnd(
        () -> {
          shooter.setShooterSpeed(ShooterConstants.Top.kInSpeed);
          shooter.setFeederSpeed(ShooterConstants.Feeder.kInSpeed);
        },
        () -> {
          shooter.setShooterSpeed(0);
          shooter.setFeederSpeed(0);
        });
  }

  public Command shootFuelReverse() {
    return Commands.startEnd(
        () -> shooter.setShooterSpeed(ShooterConstants.Top.kInSpeed),
        () -> shooter.setShooterSpeed(0));
  }

  public Command feedFuelReverse() {
    return Commands.startEnd(
        () -> shooter.setFeederSpeed(ShooterConstants.Feeder.kInSpeed),
        () -> shooter.setFeederSpeed(0));
  }

  public Command pivotShooterUp() {
    return startEnd(
        () -> shooter.setPivotSpeed(ShooterConstants.Pivot.kPivotSpeedUp),
        () -> shooter.setPivotPosition(shooter.getPivotPosition()));
  }

  public Command pivotShooterDown() {
    return startEnd(
        () -> shooter.setPivotSpeed(ShooterConstants.Pivot.kPivotSpeedDown),
        () -> shooter.setPivotPosition(shooter.getPivotPosition()));
  }

  public Command wheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return wheelSysId.quasistatic(direction);
  }

  public Command wheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return wheelSysId.dynamic(direction);
  }

  public Command feederSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return feederSysId.quasistatic(direction);
  }

  public Command feederSysIdDynamic(SysIdRoutine.Direction direction) {
    return feederSysId.dynamic(direction);
  }

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysId.quasistatic(direction);
  }

  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysId.dynamic(direction);
  }
}
