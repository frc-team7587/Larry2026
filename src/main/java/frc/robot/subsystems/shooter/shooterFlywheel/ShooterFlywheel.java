package frc.robot.subsystems.shooter.shooterFlywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterFlywheel extends SubsystemBase {
  private static final String dashboardTargetRpmKey = "Shooter/DashboardTargetRpm";
  private static final String dashboardOutputKey = "Shooter/DashboardMappedOutput";
  private final LoggedNetworkNumber Ks = new LoggedNetworkNumber("ks", 0);

  private final ShooterFlywheelIO shooter;
  private final SysIdRoutine wheelSysId;
  private double targetShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
  private double rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;

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
    setTargetShooterVelocityRpm(targetRpm);
    shooter.setShooterSpeed(speed);
  }

  private void setTargetShooterVelocityRpm(double targetRpm) {
    if (Math.abs(targetShooterVelocityRpm - targetRpm)
        > ShooterFlywheelConstants.Control.kTargetEpsilonRpm) {
      rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    }
    targetShooterVelocityRpm = targetRpm;
  }

  public void setVelocityRpm(double rpm) {
    setTargetShooterVelocityRpm(rpm);
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

  public Command runStatic() {
    return runEnd(() -> shooter.setShooterVoltage(Ks.get()), this::stopShooter);
  }

  public Command dashboardShootTune() {
    return runEnd(this::runShooterAtDashboardRpm, this::stopShooter);
  }

  public void stopShooter() {
    shooter.setShooterSpeed(ShooterFlywheelConstants.Control.kStoppedSpeed);
    targetShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
    rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    SmartDashboard.putNumber(dashboardOutputKey, 0.0);
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
    if (Math.abs(targetShooterVelocityRpm) <= ShooterFlywheelConstants.Control.kTargetEpsilonRpm
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
    double rpmError = targetShooterVelocityRpm - velocityRpm;
    boolean correctDirection = targetShooterVelocityRpm * velocityRpm > 0.0;
    boolean rpmReadyForFeed =
        Math.abs(velocityRpm)
            >= Math.abs(targetShooterVelocityRpm) - ShooterFlywheelConstants.Top.kSpeedToleranceRpm;
    boolean speedWithinTolerance = correctDirection && rpmReadyForFeed;
    if (Math.abs(targetShooterVelocityRpm) > ShooterFlywheelConstants.Control.kTargetEpsilonRpm
        && speedWithinTolerance) {
      if (rpmReadyStartTime <= ShooterFlywheelConstants.Control.kNoStableTimestamp) {
        rpmReadyStartTime = Timer.getFPGATimestamp();
      }
    } else {
      rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    }

    boolean atRpm = atRPM();
    Logger.recordOutput("Shooter/VelocityRpm", velocityRpm);
    Logger.recordOutput("Shooter/TargetVelocityRpm", targetShooterVelocityRpm);
    Logger.recordOutput("Shooter/RpmError", rpmError);
    Logger.recordOutput("Shooter/CorrectDirection", correctDirection);
    Logger.recordOutput("Shooter/RpmReadyForFeed", rpmReadyForFeed);
    Logger.recordOutput("Shooter/RpmWithinTolerance", speedWithinTolerance);
    Logger.recordOutput("Shooter/AtRPM", atRpm);
    SmartDashboard.putNumber("Shooter/MeasuredVelocityRpm", velocityRpm);
    SmartDashboard.putNumber("Shooter/CurrentTargetVelocityRpm", targetShooterVelocityRpm);
  }

  public Command wheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return wheelSysId.quasistatic(direction);
  }

  public Command wheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return wheelSysId.dynamic(direction);
  }
}
