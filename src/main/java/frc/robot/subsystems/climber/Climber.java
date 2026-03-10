package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO climber;
  private final SysIdRoutine sysId;

  public Climber(ClimberIO climber) {
    this.climber = climber;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> climber.setClimberVoltage(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climber/PositionRotations", climber.getClimberPosition());
  }

  public Command climbUp() {
    return startEnd(
        () -> climber.setClimberSpeed(ClimberConstants.kClimbUpSpeed), this::holdCurrentPosition);
  }

  public Command climbDown() {
    return startEnd(
        () -> climber.setClimberSpeed(ClimberConstants.kClimbDownSpeed), this::holdCurrentPosition);
  }

  public Command holdPosition() {
    return runOnce(this::holdCurrentPosition);
  }

  public Command stop() {
    return runOnce(() -> climber.setClimberSpeed(0.0));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  private void holdCurrentPosition() {
    climber.setClimberPosition(climber.getClimberPosition());
  }
}
