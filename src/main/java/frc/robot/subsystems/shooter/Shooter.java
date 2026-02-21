package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooter;

  public Shooter(ShooterIO shooter) {
    this.shooter = shooter;
  }

  public Command shootFuel() {
    return startEnd(
        () -> shooter.setShooterSpeed(ShooterConstants.Top.kOutSpeed),
        () -> shooter.setShooterSpeed(0));
  }

  public Command feedFuel() {
    return startEnd(
        () -> shooter.setFeederSpeed(ShooterConstants.Feeder.kOutSpeed),
        () -> shooter.setFeederSpeed(0));
  }
}
