package frc.robot.subsystems.shooter.ShooterFlywheel;

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
  private int speedDipCounterLoops = 0;
  private boolean speedDipActive = false;
  private double lastSpeedDipTimestampSec = ShooterFlywheelConstants.Control.kNoStableTimestamp;
  private double lastSpeedDipMagnitudeRpm = 0.0;

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
    speedDipCounterLoops = 0;
    speedDipActive = false;
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
    double absVelocityRpm = Math.abs(velocityRpm);
    double absTargetRpm = Math.abs(targetShooterVelocityRpm);
    double rpmError = targetShooterVelocityRpm - velocityRpm;
    boolean correctDirection = targetShooterVelocityRpm * velocityRpm > 0.0;
    boolean rpmReadyForFeed =
        absVelocityRpm >= absTargetRpm - ShooterFlywheelConstants.Top.kSpeedToleranceRpm;
    boolean speedWithinTolerance = correctDirection && rpmReadyForFeed;
    if (Math.abs(targetShooterVelocityRpm) > ShooterFlywheelConstants.Control.kTargetEpsilonRpm
        && speedWithinTolerance) {
      if (rpmReadyStartTime <= ShooterFlywheelConstants.Control.kNoStableTimestamp) {
        rpmReadyStartTime = Timer.getFPGATimestamp();
      }
    } else {
      rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    }

    double dipThresholdRpm =
        Math.max(
            ShooterFlywheelConstants.Control.kDipToleranceRpm,
            absTargetRpm * ShooterFlywheelConstants.Control.kDipToleranceFraction);
    double speedDeficitRpm = absTargetRpm - absVelocityRpm;
    boolean dipSample =
        absTargetRpm >= ShooterFlywheelConstants.Control.kDipMinTargetRpm
            && correctDirection
            && speedDeficitRpm >= dipThresholdRpm;
    if (dipSample) {
      speedDipCounterLoops++;
    } else {
      speedDipCounterLoops = 0;
    }

    boolean speedDipNow =
        speedDipCounterLoops >= ShooterFlywheelConstants.Control.kDipDebounceLoops;
    boolean speedDipRisingEdge = speedDipNow && !speedDipActive;
    if (speedDipRisingEdge) {
      lastSpeedDipTimestampSec = Timer.getFPGATimestamp();
      lastSpeedDipMagnitudeRpm = speedDeficitRpm;
    }
    speedDipActive = speedDipNow;

    boolean atRpm = atRPM();
    Logger.recordOutput("Shooter/VelocityRpm", velocityRpm);
    Logger.recordOutput("Shooter/TargetVelocityRpm", targetShooterVelocityRpm);
    Logger.recordOutput("Shooter/RpmError", rpmError);
    Logger.recordOutput("Shooter/CorrectDirection", correctDirection);
    Logger.recordOutput("Shooter/RpmReadyForFeed", rpmReadyForFeed);
    Logger.recordOutput("Shooter/RpmWithinTolerance", speedWithinTolerance);
    Logger.recordOutput("Shooter/AtRPM", atRpm);
    Logger.recordOutput("Shooter/SpeedDipThresholdRpm", dipThresholdRpm);
    Logger.recordOutput("Shooter/SpeedDeficitRpm", speedDeficitRpm);
    Logger.recordOutput("Shooter/SpeedDipCounterLoops", speedDipCounterLoops);
    Logger.recordOutput("Shooter/SpeedDipSample", dipSample);
    Logger.recordOutput("Shooter/SpeedDipActive", speedDipActive);
    Logger.recordOutput("Shooter/SpeedDipRisingEdge", speedDipRisingEdge);
    Logger.recordOutput("Shooter/SpeedDipLastTimestampSec", lastSpeedDipTimestampSec);
    Logger.recordOutput("Shooter/SpeedDipLastMagnitudeRpm", lastSpeedDipMagnitudeRpm);
    SmartDashboard.putNumber("Shooter/MeasuredVelocityRpm", velocityRpm);
    SmartDashboard.putNumber("Shooter/CurrentTargetVelocityRpm", targetShooterVelocityRpm);
    SmartDashboard.putNumber("Shooter/SpeedDeficitRpm", speedDeficitRpm);
    SmartDashboard.putBoolean("Shooter/SpeedDipActive", speedDipActive);
    SmartDashboard.putNumber("Shooter/SpeedDipLastTimestampSec", lastSpeedDipTimestampSec);
    SmartDashboard.putNumber("Shooter/SpeedDipLastMagnitudeRpm", lastSpeedDipMagnitudeRpm);
  }

  public Command wheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return wheelSysId.quasistatic(direction);
  }

  public Command wheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return wheelSysId.dynamic(direction);
  }
}
