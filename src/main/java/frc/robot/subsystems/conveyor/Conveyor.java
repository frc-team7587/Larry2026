package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  private final ConveyorIO conveyor;

  public Conveyor(ConveyorIO conveyor) {
    this.conveyor = conveyor;
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
}
