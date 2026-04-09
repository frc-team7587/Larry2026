package frc.robot.subsystems.shooter.ShooterFlywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterFlywheel extends SubsystemBase {
  private static final String dashboardTargetRpmKey = "Shooter/DashboardTargetRpm";
  private static final String dashboardOutputKey = "Shooter/DashboardMappedOutput";
  private static final String measuredVelocityRpmKey = "Shooter/MeasuredVelocityRpm";
  private static final String currentTargetVelocityRpmKey = "Shooter/CurrentTargetVelocityRpm";
  private static final String speedDeficitRpmKey = "Shooter/SpeedDeficitRpm";
  private static final String speedDipActiveKey = "Shooter/SpeedDipActive";
  private static final String speedDipLastTimestampSecKey = "Shooter/SpeedDipLastTimestampSec";
  private static final String speedDipLastMagnitudeRpmKey = "Shooter/SpeedDipLastMagnitudeRpm";
  private final LoggedNetworkNumber Ks = new LoggedNetworkNumber("ks", 0);

  private final ShooterFlywheelIO io;
  private final SysIdRoutine wheelSysId;
  private double targetShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
  private double rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
  private int speedDipCounterLoops = 0;
  private boolean speedDipActive = false;
  private double lastSpeedDipTimestampSec = ShooterFlywheelConstants.Control.kNoStableTimestamp;
  private double lastSpeedDipMagnitudeRpm = 0.0;

  private record ShooterTelemetry(
      double velocityRpm,
      double targetVelocityRpm,
      double rpmError,
      boolean correctDirection,
      boolean rpmReadyForFeed,
      boolean speedWithinTolerance,
      double dipThresholdRpm,
      double speedDeficitRpm,
      boolean dipSample,
      boolean speedDipRisingEdge) {}

  public ShooterFlywheel(ShooterFlywheelIO shooter) {
    this.io = shooter;
    wheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/WheelSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setShooterVoltage(voltage.in(Volts)), null, this));

    SmartDashboard.putNumber(
        dashboardTargetRpmKey, ShooterFlywheelConstants.Control.kDashboardDefaultTargetRpm);
    SmartDashboard.putNumber(dashboardOutputKey, 0.0);
  }

  private void runPercentOutput(double speed, double targetRpm) {
    setTargetShooterVelocityRpm(targetRpm);
    io.setShooterSpeed(speed);
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
    io.setVelocity(rpm);
  }

  public Command setVelocityCommand(double rpm) {
    return startEnd(() -> setVelocityRpm(rpm), this::stopShooter);
  }

  private double getDashboardTargetRpm() {
    return SmartDashboard.getNumber(
        dashboardTargetRpmKey, ShooterFlywheelConstants.Control.kDashboardDefaultTargetRpm);
  }

  public void runShooterAtDashboardRpm() {
    double requestedRpm = getDashboardTargetRpm();
    setVelocityRpm(requestedRpm);
    Logger.recordOutput("Shooter/DashboardRequestedTargetRpm", requestedRpm);
    Logger.recordOutput(
        "Shooter/DashboardRequestedTargetRpmRadPerSec", requestedRpm * 2 * Math.PI / 60.0);
  }

  public Command runStatic() {
    return runEnd(() -> io.setShooterVoltage(Ks.get()), this::stopShooter);
  }

  public Command dashboardShootTune() {
    return runEnd(this::runShooterAtDashboardRpm, this::stopShooter);
  }

  public void stopShooter() {
    io.setShooterSpeed(ShooterFlywheelConstants.Control.kStoppedSpeed);
    resetControlState();
    SmartDashboard.putNumber(dashboardOutputKey, 0.0);
  }

  public Command shootFuel() {
    return createPercentOutputCommand(
        ShooterFlywheelConstants.Top.kOutSpeed, ShooterFlywheelConstants.Top.kOutTargetRpm);
  }

  public Command shootFuelAtRPM(double targetRpm) {
    return createPercentOutputCommand(ShooterFlywheelConstants.Top.kOutSpeed, targetRpm);
  }

  public Command shootFuelReverse() {
    return createPercentOutputCommand(
        ShooterFlywheelConstants.Top.kInSpeed, ShooterFlywheelConstants.Top.kInTargetRpm);
  }

  private Command createPercentOutputCommand(double speed, double targetRpm) {
    return startEnd(() -> runPercentOutput(speed, targetRpm), this::stopShooter);
  }

  @AutoLogOutput(key = "Shooter/AtRPM")
  public boolean atRPM() {
    return isVelocityWithinTolerance(targetShooterVelocityRpm, io.getShooterVelocityRpm());
  }

  public double getShooterVelocityRpm() {
    return io.getShooterVelocityRpm();
  }

  @Override
  public void periodic() {
    ShooterTelemetry telemetry = updateTelemetry(io.getShooterVelocityRpm());
    logTelemetry(telemetry);
    updateDashboard(telemetry);
  }

  private ShooterTelemetry updateTelemetry(double velocityRpm) {
    double absVelocityRpm = Math.abs(velocityRpm);
    double absTargetRpm = Math.abs(targetShooterVelocityRpm);
    double rpmError = targetShooterVelocityRpm - velocityRpm;
    boolean correctDirection = targetShooterVelocityRpm * velocityRpm > 0.0;
    boolean rpmReadyForFeed = isAbsoluteVelocityWithinTolerance(absTargetRpm, absVelocityRpm);
    boolean speedWithinTolerance = correctDirection && rpmReadyForFeed;

    updateRpmReadyState(absTargetRpm, speedWithinTolerance);

    double dipThresholdRpm = getDipThresholdRpm(absTargetRpm);
    double speedDeficitRpm = absTargetRpm - absVelocityRpm;
    boolean dipSample =
        isDipSample(absTargetRpm, correctDirection, speedDeficitRpm, dipThresholdRpm);
    boolean speedDipRisingEdge = updateSpeedDipState(dipSample, speedDeficitRpm);

    return new ShooterTelemetry(
        velocityRpm,
        targetShooterVelocityRpm,
        rpmError,
        correctDirection,
        rpmReadyForFeed,
        speedWithinTolerance,
        dipThresholdRpm,
        speedDeficitRpm,
        dipSample,
        speedDipRisingEdge);
  }

  private void updateRpmReadyState(double absTargetRpm, boolean speedWithinTolerance) {
    if (absTargetRpm > ShooterFlywheelConstants.Control.kTargetEpsilonRpm && speedWithinTolerance) {
      if (rpmReadyStartTime <= ShooterFlywheelConstants.Control.kNoStableTimestamp) {
        rpmReadyStartTime = Timer.getFPGATimestamp();
      }
      return;
    }

    rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
  }

  private double getDipThresholdRpm(double absTargetRpm) {
    return Math.max(
        ShooterFlywheelConstants.Control.kDipToleranceRpm,
        absTargetRpm * ShooterFlywheelConstants.Control.kDipToleranceFraction);
  }

  private boolean isDipSample(
      double absTargetRpm,
      boolean correctDirection,
      double speedDeficitRpm,
      double dipThresholdRpm) {
    return absTargetRpm >= ShooterFlywheelConstants.Control.kDipMinTargetRpm
        && correctDirection
        && speedDeficitRpm >= dipThresholdRpm;
  }

  private boolean updateSpeedDipState(boolean dipSample, double speedDeficitRpm) {
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
    return speedDipRisingEdge;
  }

  private void logTelemetry(ShooterTelemetry telemetry) {
    Logger.recordOutput("Shooter/VelocityRpm", telemetry.velocityRpm());
    Logger.recordOutput("Shooter/TargetVelocityRpm", telemetry.targetVelocityRpm());
    Logger.recordOutput("Shooter/RpmError", telemetry.rpmError());
    Logger.recordOutput("Shooter/CorrectDirection", telemetry.correctDirection());
    Logger.recordOutput("Shooter/RpmReadyForFeed", telemetry.rpmReadyForFeed());
    Logger.recordOutput("Shooter/RpmWithinTolerance", telemetry.speedWithinTolerance());
    Logger.recordOutput("Shooter/SpeedDipThresholdRpm", telemetry.dipThresholdRpm());
    Logger.recordOutput("Shooter/SpeedDeficitRpm", telemetry.speedDeficitRpm());
    Logger.recordOutput("Shooter/SpeedDipCounterLoops", speedDipCounterLoops);
    Logger.recordOutput("Shooter/SpeedDipSample", telemetry.dipSample());
    Logger.recordOutput("Shooter/SpeedDipActive", speedDipActive);
    Logger.recordOutput("Shooter/SpeedDipRisingEdge", telemetry.speedDipRisingEdge());
    Logger.recordOutput("Shooter/SpeedDipLastTimestampSec", lastSpeedDipTimestampSec);
    Logger.recordOutput("Shooter/SpeedDipLastMagnitudeRpm", lastSpeedDipMagnitudeRpm);
  }

  private void updateDashboard(ShooterTelemetry telemetry) {
    SmartDashboard.putNumber(measuredVelocityRpmKey, telemetry.velocityRpm());
    SmartDashboard.putNumber(currentTargetVelocityRpmKey, telemetry.targetVelocityRpm());
    SmartDashboard.putNumber(speedDeficitRpmKey, telemetry.speedDeficitRpm());
    SmartDashboard.putBoolean(speedDipActiveKey, speedDipActive);
    SmartDashboard.putNumber(speedDipLastTimestampSecKey, lastSpeedDipTimestampSec);
    SmartDashboard.putNumber(speedDipLastMagnitudeRpmKey, lastSpeedDipMagnitudeRpm);
  }

  private void resetControlState() {
    targetShooterVelocityRpm = ShooterFlywheelConstants.Control.kNoTargetRpm;
    rpmReadyStartTime = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    speedDipCounterLoops = 0;
    speedDipActive = false;
    lastSpeedDipTimestampSec = ShooterFlywheelConstants.Control.kNoStableTimestamp;
    lastSpeedDipMagnitudeRpm = 0.0;
  }

  private boolean isVelocityWithinTolerance(double targetRpm, double velocityRpm) {
    return Math.abs(targetRpm - velocityRpm) < ShooterFlywheelConstants.Top.kSpeedToleranceRpm;
  }

  private boolean isAbsoluteVelocityWithinTolerance(double targetRpm, double velocityRpm) {
    return velocityRpm >= targetRpm - ShooterFlywheelConstants.Top.kSpeedToleranceRpm;
  }

  public Command wheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return wheelSysId.quasistatic(direction);
  }

  public Command wheelSysIdDynamic(SysIdRoutine.Direction direction) {
    return wheelSysId.dynamic(direction);
  }
}
