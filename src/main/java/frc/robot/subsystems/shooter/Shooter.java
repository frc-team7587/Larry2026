package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooter;
  private final SysIdRoutine wheelSysId;
  private final SysIdRoutine pivotSysId;
  private double targetShooterVelocityRpm = ShooterConstants.Control.kNoTargetRpm;
  private double speedWithinToleranceStartTime = ShooterConstants.Control.kNoStableTimestamp;

  public Shooter(ShooterIO shooter) {
    this.shooter = shooter;
    wheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/WheelSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> shooter.setShooterVoltage(voltage.in(Volts)), null, this));
    pivotSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/PivotSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> shooter.setPivotVoltage(voltage.in(Volts)), null, this));
  }

  public void setShooterSpeedWithTargetRpm(double speed, double targetRpm) {
    targetShooterVelocityRpm = targetRpm;
    speedWithinToleranceStartTime = ShooterConstants.Control.kNoStableTimestamp;
    shooter.setShooterSpeed(speed);
  }

  public void setPivotPosition(double position) {
    shooter.setPivotPosition(position);
  }

  public void holdPivotPosition() {
    shooter.setPivotPosition(shooter.getPivotPosition());
  }

  public void stopShooter() {
    shooter.setShooterSpeed(ShooterConstants.Control.kStoppedSpeed);
    targetShooterVelocityRpm = ShooterConstants.Control.kNoTargetRpm;
    speedWithinToleranceStartTime = ShooterConstants.Control.kNoStableTimestamp;
  }

  public Command shootFuel() {
    return startEnd(
        () -> {
          setShooterSpeedWithTargetRpm(
              ShooterConstants.Top.kOutSpeed, ShooterConstants.Top.kOutTargetRpm);
        },
        this::stopShooter);
  }

  public Command shootFuelReverse() {
    return startEnd(
        () -> {
          setShooterSpeedWithTargetRpm(
              ShooterConstants.Top.kInSpeed, ShooterConstants.Top.kInTargetRpm);
        },
        this::stopShooter);
  }

  public boolean atSpeed() {
    if (Math.abs(targetShooterVelocityRpm) <= ShooterConstants.Control.kTargetEpsilonRpm
        || speedWithinToleranceStartTime < ShooterConstants.Control.kNoStableTimestamp) {
      return false;
    }
    return Timer.getFPGATimestamp() - speedWithinToleranceStartTime
        >= ShooterConstants.Top.kSpeedStableTimeSec;
  }

  public double getShooterVelocityRpm() {
    return shooter.getShooterVelocityRpm();
  }

  @Override
  public void periodic() {
    double velocityRpm = shooter.getShooterVelocityRpm();
    boolean speedWithinTolerance =
        Math.abs(targetShooterVelocityRpm - velocityRpm) <= ShooterConstants.Top.kSpeedToleranceRpm;
    if (Math.abs(targetShooterVelocityRpm) > ShooterConstants.Control.kTargetEpsilonRpm
        && speedWithinTolerance) {
      if (speedWithinToleranceStartTime < ShooterConstants.Control.kNoStableTimestamp) {
        speedWithinToleranceStartTime = Timer.getFPGATimestamp();
      }
    } else {
      speedWithinToleranceStartTime = ShooterConstants.Control.kNoStableTimestamp;
    }

    Logger.recordOutput("Shooter/VelocityRpm", velocityRpm);
    Logger.recordOutput("Shooter/TargetVelocityRpm", targetShooterVelocityRpm);
    Logger.recordOutput("Shooter/AtSpeed", atSpeed());
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

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotSysId.quasistatic(direction);
  }

  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotSysId.dynamic(direction);
  }
}
