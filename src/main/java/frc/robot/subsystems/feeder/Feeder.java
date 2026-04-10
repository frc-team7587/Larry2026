package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Feeder extends SubsystemBase {
  private static final double kVelocityEpsilonRpm = 1e-3;

  private final FeederIO feeder;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final SysIdRoutine sysId;
  private double appliedVelocityRpm = 0.0;
  private boolean idleArmed = false;

  public static final LoggedNetworkNumber KS = new LoggedNetworkNumber("Feeder/KS", 0.0);

  public Feeder(FeederIO feeder) {
    this.feeder = feeder;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Feeder/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> feeder.setFeederVoltage(voltage.in(Volts)), null, this));
  }

  public Command feedFuel() {
    return startEnd(() -> setVelocityRpm(FeederConstants.kOutTargetRpm), this::stop);
  }

  public Command feedFuelReverse() {
    return startEnd(() -> setVelocityRpm(FeederConstants.kInTargetRpm), this::stop);
  }

  // Tuning only - managed in Elastic
  public Command runStatic() {
    return runEnd(() -> feeder.setFeederVoltage(KS.get()), this::hardStop);
  }

  public void setVelocityRpm(double rpm) {
    if (Math.abs(rpm) > kVelocityEpsilonRpm) {
      idleArmed = true;
    }
    applyVelocity(idleSetpointFor(rpm));
  }

  public void stop() {
    applyVelocity(idleSetpointFor(0.0));
  }

  public void hardStop() {
    applyVelocity(0.0);
  }

  private double idleSetpointFor(double requestedRpm) {
    if (Math.abs(requestedRpm) > kVelocityEpsilonRpm) {
      return requestedRpm;
    }
    return idleArmed ? FeederConstants.kIdleTargetRpm : 0.0;
  }

  private void applyVelocity(double rpm) {
    appliedVelocityRpm = rpm;
    feeder.setVelocity(rpm);
  }

  @Override
  public void periodic() {
    feeder.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  @AutoLogOutput(key = "Feeder/TargetVelocityRpm")
  public double getAppliedVelocityRpm() {
    return appliedVelocityRpm;
  }

  @AutoLogOutput(key = "Feeder/IdleArmed")
  public boolean isIdleArmed() {
    return idleArmed;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
