package frc.robot.subsystems.shooter.shooterFlywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class ShooterFlywheel extends SubsystemBase {
  private static final String dashboardTargetRpmKey = "Shooter/DashboardTargetRpm";
  private static final String dashboardOutputKey = "Shooter/DashboardMappedOutput";

  private final ShooterFlywheelIO shooter;
  private final SysIdRoutine wheelSysId;
  private double activeTargetShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
  private double appliedShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
  private double rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
  private boolean idleEnabled = ShooterFlywheelConstants.Control.kEnableIdleAfterFirstSpinup;
  private boolean idleArmed = false;
  private boolean idleActive = false;

  public ShooterFlywheel(ShooterFlywheelIO shooter) {
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

    SmartDashboard.putNumber(
        dashboardTargetRpmKey, ShooterFlywheelConstants.Control.kDashboardDefaultTargetRpm);
    SmartDashboard.putNumber(dashboardOutputKey, 0.0);
  }

  public void setShooterSpeedWithTargetRpm(double speed, double targetRpm) {
    setActiveTargetShooterVelocityRpm(targetRpm);
    appliedShooterVelocityRpm = targetRpm;
    armIdleIfSpunUp(targetRpm);
    idleActive = false;
    shooter.setShooterSpeed(speed);
  }

  private void setActiveTargetShooterVelocityRpm(double targetRpm) {
    if (Math.abs(activeTargetShooterVelocityRpm - targetRpm)
        > ShooterFlywheelConstants.Control.kTargetEpsilonRpm) {
      rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    }
    activeTargetShooterVelocityRpm = targetRpm;
  }

  public void setVelocityRpm(double rpm) {
    setActiveTargetShooterVelocityRpm(rpm);
    appliedShooterVelocityRpm = rpm;
    armIdleIfSpunUp(rpm);
    idleActive = false;
    shooter.setVelocity(rpm);
  }

  public Command setVelocityCommand(double rpm) {
    return startEnd(() -> setVelocityRpm(rpm), this::stopShooter);
  }

  public void runShooterAtDashboardRpm() {
    double requestedRpm =
        SmartDashboard.getNumber(
            dashboardTargetRpmKey, ShooterFlywheelConstants.Control.kDashboardDefaultTargetRpm);

    setVelocityRpm(requestedRpm);
    Logger.recordOutput("Shooter/DashboardRequestedTargetRpm", requestedRpm);
    Logger.recordOutput(
        "Shooter/DashboardRequestedTargetRpmRadPerSec", requestedRpm * 2 * Math.PI / 60.0);
  }

  public Command dashboardShootTune() {
    return runEnd(this::runShooterAtDashboardRpm, this::stopShooter);
  }

  public void stopShooter() {
    activeTargetShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
    rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    SmartDashboard.putNumber(dashboardOutputKey, 0.0);

    if (shouldRunIdle()) {
      appliedShooterVelocityRpm = ShooterFlywheelConstants.Control.kIdleTargetRpm;
      idleActive = true;
      shooter.setVelocity(appliedShooterVelocityRpm);
      return;
    }

    appliedShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
    idleActive = false;
    shooter.setShooterSpeed(ShooterFlywheelConstants.Control.kStoppedSpeed);
  }

  public void toggleIdleEnabled() {
    setIdleEnabled(!idleEnabled);
  }

  public void setIdleEnabled(boolean enabled) {
    idleEnabled = enabled;
    Logger.recordOutput("Shooter/IdleEnabled", idleEnabled);

    if (!idleEnabled && idleActive) {
      appliedShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
      idleActive = false;
      shooter.setShooterSpeed(ShooterFlywheelConstants.Control.kStoppedSpeed);
    }
  }

  public Command shootFuel() {
    return startEnd(
        () -> {
          setShooterSpeedWithTargetRpm(
              ShooterFlywheelConstants.Top.kOutSpeed, ShooterFlywheelConstants.Top.kOutTargetRpm);
        },
        this::stopShooter);
  }

  public Command shootFuelAtRPM(double targetRpm) {
    return startEnd(
        () -> {
          setShooterSpeedWithTargetRpm(ShooterFlywheelConstants.Top.kOutSpeed, targetRpm);
        },
        this::stopShooter);
  }

  public Command shootFuelReverse() {
    return startEnd(
        () -> {
          setShooterSpeedWithTargetRpm(
              ShooterFlywheelConstants.Top.kInSpeed, ShooterFlywheelConstants.Top.kInTargetRpm);
        },
        this::stopShooter);
  }

  public boolean atRPM() {
    if (Math.abs(activeTargetShooterVelocityRpm)
            <= ShooterFlywheelConstants.Control.kTargetEpsilonRpm
        || rpmReadyStartTime <= ShooterFlywheelConstants.Control.kNoStableTimestamp) {
      return false;
    }
    return Timer.getFPGATimestamp() - rpmReadyStartTime
        >= ShooterFlywheelConstants.Top.kSpeedStableTimeSec;
  }

  public double getShooterVelocityRpm() {
    return shooter.getShooterVelocityRpm();
  }

  @Override
  public void periodic() {
    double velocityRpm = shooter.getShooterVelocityRpm();
    double rpmError = activeTargetShooterVelocityRpm - velocityRpm;
    boolean correctDirection = activeTargetShooterVelocityRpm * velocityRpm > 0.0;
    boolean rpmReadyForFeed =
        Math.abs(velocityRpm)
            >= Math.abs(activeTargetShooterVelocityRpm)
                - ShooterFlywheelConstants.Top.kSpeedToleranceRpm;
    boolean speedWithinTolerance = correctDirection && rpmReadyForFeed;
    if (Math.abs(activeTargetShooterVelocityRpm)
            > ShooterFlywheelConstants.Control.kTargetEpsilonRpm
        && speedWithinTolerance) {
      if (rpmReadyStartTime <= ShooterFlywheelConstants.Control.kNoStableTimestamp) {
        rpmReadyStartTime = Timer.getFPGATimestamp();
      }
    } else {
      rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    }

    boolean atRpm = atRPM();
    Logger.recordOutput("Shooter/VelocityRpm", velocityRpm);
    Logger.recordOutput("Shooter/TargetVelocityRpm", appliedShooterVelocityRpm);
    Logger.recordOutput("Shooter/ActiveTargetVelocityRpm", activeTargetShooterVelocityRpm);
    Logger.recordOutput("Shooter/RpmError", rpmError);
    Logger.recordOutput("Shooter/CorrectDirection", correctDirection);
    Logger.recordOutput("Shooter/RpmReadyForFeed", rpmReadyForFeed);
    Logger.recordOutput("Shooter/RpmWithinTolerance", speedWithinTolerance);
    Logger.recordOutput("Shooter/AtRPM", atRpm);
    Logger.recordOutput("Shooter/IdleEnabled", idleEnabled);
    Logger.recordOutput("Shooter/IdleArmed", idleArmed);
    Logger.recordOutput("Shooter/IdleActive", idleActive);
    SmartDashboard.putNumber("Shooter/MeasuredVelocityRpm", velocityRpm);
    SmartDashboard.putNumber("Shooter/CurrentTargetVelocityRpm", appliedShooterVelocityRpm);
  }

  private boolean shouldRunIdle() {
    return idleEnabled && idleArmed;
  }

  private void armIdleIfSpunUp(double rpm) {
    if (Math.abs(rpm) > ShooterFlywheelConstants.Control.kTargetEpsilonRpm) {
      idleArmed = true;
    }
  }

  public Command wheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return wheelSysId.quasistatic(direction);
  }

  public Command wheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return wheelSysId.dynamic(direction);
  }
}
