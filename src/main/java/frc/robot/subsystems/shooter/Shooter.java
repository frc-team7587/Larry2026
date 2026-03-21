package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final String dashboardTargetRpmKey = "Shooter/DashboardTargetRpm";
  private static final String dashboardOutputKey = "Shooter/DashboardMappedOutput";

  private final ShooterIO shooter;
  private final SysIdRoutine wheelSysId;
  private final SysIdRoutine pivotSysId;
  private double targetShooterVelocityRpm = ShooterConstants.Control.kNoTargetRpm;
  private double rpmReadyStartTime = ShooterConstants.Control.kNoStableTimestamp;

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

    shooter.setPivotEncoderPosition(ShooterConstants.Pivot.kBottomPosition);
    SmartDashboard.putNumber(
        dashboardTargetRpmKey, ShooterConstants.Control.kDashboardDefaultTargetRpm);
    SmartDashboard.putNumber(dashboardOutputKey, 0.0);
  }

  public void setShooterSpeedWithTargetRpm(double speed, double targetRpm) {

    targetShooterVelocityRpm = targetRpm;
    rpmReadyStartTime = ShooterConstants.Control.kNoStableTimestamp;
    shooter.setShooterSpeed(speed);
  }

  public void setVelocityRobot(double rpm) {
    shooter.setVelocity(rpm);
  }

  public Command setVelocityCommand(double rpm) {
    return startEnd(() -> setVelocityRobot(rpm), () -> setVelocityRobot(0));
  }

  public void runShooterAtDashboardRpm() {
    double requestedRpm =
        SmartDashboard.getNumber(
            dashboardTargetRpmKey, ShooterConstants.Control.kDashboardDefaultTargetRpm);
    double clampedRpm =
        MathUtil.clamp(
            requestedRpm,
            -ShooterConstants.Control.kDashboardMaxTargetRpm,
            ShooterConstants.Control.kDashboardMaxTargetRpm);
    double mappedOutput = clampedRpm / ShooterConstants.Control.kDashboardMaxTargetRpm;

    setShooterSpeedWithTargetRpm(mappedOutput, clampedRpm);
    SmartDashboard.putNumber(dashboardOutputKey, mappedOutput);
    Logger.recordOutput("Shooter/DashboardRequestedTargetRpm", requestedRpm);
    Logger.recordOutput("Shooter/DashboardClampedTargetRpm", clampedRpm);
    Logger.recordOutput("Shooter/DashboardMappedOutput", mappedOutput);
  }

  public Command dashboardShootTune() {
    return runEnd(this::runShooterAtDashboardRpm, this::stopShooter);
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

  public void stopShooter() {
    shooter.setShooterSpeed(ShooterConstants.Control.kStoppedSpeed);
    targetShooterVelocityRpm = ShooterConstants.Control.kNoTargetRpm;
    rpmReadyStartTime = ShooterConstants.Control.kNoStableTimestamp;
    SmartDashboard.putNumber(dashboardOutputKey, 0.0);
  }

  public Command shootFuel() {
    return startEnd(
        () -> {
          setShooterSpeedWithTargetRpm(
              ShooterConstants.Top.kOutSpeed, ShooterConstants.Top.kOutTargetRpm);
        },
        this::stopShooter);
  }

  public Command shootFuelAtRPM(double targetRpm) {
    return startEnd(
        () -> {
          setShooterSpeedWithTargetRpm(ShooterConstants.Top.kOutSpeed, targetRpm);
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

  public boolean atRPM() {
    if (Math.abs(targetShooterVelocityRpm) <= ShooterConstants.Control.kTargetEpsilonRpm
        || rpmReadyStartTime <= ShooterConstants.Control.kNoStableTimestamp) {
      return false;
    }
    return Timer.getFPGATimestamp() - rpmReadyStartTime >= ShooterConstants.Top.kSpeedStableTimeSec;
  }

  public double getShooterVelocityRpm() {
    return shooter.getShooterVelocityRpm();
  }

  public double getPivotPosition() {
    return shooter.getPivotPosition();
  }

  @Override
  public void periodic() {
    double velocityRpm = shooter.getShooterVelocityRpm();
    double pivotPosition = shooter.getPivotPosition();
    double rpmError = targetShooterVelocityRpm - velocityRpm;
    boolean correctDirection = targetShooterVelocityRpm * velocityRpm > 0.0;
    boolean rpmReadyForFeed =
        Math.abs(velocityRpm)
            >= Math.abs(targetShooterVelocityRpm) - ShooterConstants.Top.kSpeedToleranceRpm;
    boolean speedWithinTolerance = correctDirection && rpmReadyForFeed;
    if (Math.abs(targetShooterVelocityRpm) > ShooterConstants.Control.kTargetEpsilonRpm
        && speedWithinTolerance) {
      if (rpmReadyStartTime <= ShooterConstants.Control.kNoStableTimestamp) {
        rpmReadyStartTime = Timer.getFPGATimestamp();
      }
    } else {
      rpmReadyStartTime = ShooterConstants.Control.kNoStableTimestamp;
    }

    boolean atRpm = atRPM();
    Logger.recordOutput("Shooter/PivotEncoderPosition", pivotPosition);
    Logger.recordOutput("Shooter/PivotPosition", pivotPosition);
    Logger.recordOutput("Shooter/VelocityRpm", velocityRpm);
    Logger.recordOutput("Shooter/TargetVelocityRpm", targetShooterVelocityRpm);
    Logger.recordOutput("Shooter/RpmError", rpmError);
    Logger.recordOutput("Shooter/CorrectDirection", correctDirection);
    Logger.recordOutput("Shooter/RpmReadyForFeed", rpmReadyForFeed);
    Logger.recordOutput("Shooter/RpmWithinTolerance", speedWithinTolerance);
    Logger.recordOutput("Shooter/AtRPM", atRpm);
    Logger.recordOutput("Shooter/atRPM", atRpm);
    SmartDashboard.putNumber("Shooter/MeasuredVelocityRpm", velocityRpm);
    SmartDashboard.putNumber("Shooter/CurrentTargetVelocityRpm", targetShooterVelocityRpm);
    SmartDashboard.putNumber("Shooter/PivotEncoderPosition", pivotPosition);
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
