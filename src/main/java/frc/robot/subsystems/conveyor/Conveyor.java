package frc.robot.subsystems.conveyor;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO conveyor;
  private final SysIdRoutine sysId;

  public Conveyor(ConveyorIO conveyor) {
    this.conveyor = conveyor;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Conveyor/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> conveyor.setConveyorVoltage(voltage.in(Volts)), null, this));
  }

  public Command transportBalls() {
    return startEnd(
        () -> conveyor.setConveyorSpeed(ConveyorConstants.kForwardsSpeed),
        () -> conveyor.setConveyorSpeed(0));
  }

  public Command transportBallsReverse() {
    return startEnd(
        () -> conveyor.setConveyorSpeed(ConveyorConstants.kBackwardsSpeed),
        () -> conveyor.setConveyorSpeed(0));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
