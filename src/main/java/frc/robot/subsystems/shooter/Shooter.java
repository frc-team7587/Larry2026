package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooter;

  public Shooter(ShooterIO shooter) {
    this.shooter = shooter;
  }

  public Command shootFuel() {
    return Commands.startEnd(
        () -> shooter.setShooterSpeed(ShooterConstants.Top.kOutSpeed),
        () -> shooter.setShooterSpeed(0));
  }

  public Command feedFuel() {
    return Commands.startEnd(
        () -> shooter.setFeederSpeed(ShooterConstants.Feeder.kOutSpeed),
        () -> shooter.setFeederSpeed(0));
  }

  public Command shootAndFeedFuel() {
    return Commands.startEnd(
        () -> {
          shooter.setShooterSpeed(ShooterConstants.Top.kOutSpeed);
          shooter.setFeederSpeed(ShooterConstants.Feeder.kOutSpeed);
        },
        () -> {
          shooter.setShooterSpeed(0);
          shooter.setFeederSpeed(0);
        });
  }

  public Command shootFuelReverse() {
    return Commands.startEnd(
        () -> shooter.setShooterSpeed(ShooterConstants.Top.kInSpeed),
        () -> shooter.setShooterSpeed(0));
  }

  public Command feedFuelReverse() {
    return Commands.startEnd(
        () -> shooter.setFeederSpeed(ShooterConstants.Feeder.kInSpeed),
        () -> shooter.setFeederSpeed(0));
  }

  public Command pivotShooterUp() {
    return startEnd(
        () -> shooter.setPivotSpeed(ShooterConstants.Pivot.kPivotSpeedUp),
        () -> shooter.setPivotSpeed(0));
  }

  public Command pivotShooterDown() {
    return startEnd(
        () -> shooter.setPivotSpeed(ShooterConstants.Pivot.kPivotSpeedDown),
        () -> shooter.setPivotSpeed(0));
  }
}
