package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO feeder;
  private final SysIdRoutine sysId;

  public Feeder(FeederIO feeder) {
    this.feeder = feeder;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> Logger.recordOutput("Feeder/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> feeder.setFeederVoltage(voltage.in(Volts)), null, this));
  }

  public Command feedFuel() {
    return startEnd(
        () -> feeder.setFeederSpeed(FeederConstants.kOutSpeed), () -> feeder.setFeederSpeed(0));
  }

  public Command feedFuelReverse() {
    return startEnd(
        () -> feeder.setFeederSpeed(FeederConstants.kInSpeed), () -> feeder.setFeederSpeed(0));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
