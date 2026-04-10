package frc.robot.subsystems.intake.IntakeFlywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeFlywheel extends SubsystemBase {
  // enum tree on state
  private enum IntakeState {
    STOPPED,
    INTAKING,
    OUTTAKING
  }

  private final IntakeFlywheelIO intake;
  private final IntakeFlywheelIOInputsAutoLogged inputs = new IntakeFlywheelIOInputsAutoLogged();
  private final SysIdRoutine intakeSysId;
  private double commandedSpeed = 0.0;
  private boolean boostActive = false; // never start w/ boost active lol
  private IntakeState intakeState = IntakeState.STOPPED; // start w/ state-lock

  public IntakeFlywheel(IntakeFlywheelIO intake) {
    this.intake = intake;
    intakeSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/RollerSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> intake.setIntakeVoltage(voltage.in(Volts)), null, this));
  }

  public void setIntakeSpeed(double speed) {
    commandedSpeed =
        MathUtil.clamp(
            speed,
            IntakeFlywheelConstants.Intake.kMinOutput,
            IntakeFlywheelConstants.Intake.kMaxOutput);
    intake.setIntakeSpeed(commandedSpeed);
    boostActive = false;
    updateStateFromSpeed(commandedSpeed);
  }

  public Command intakeFuel() {
    return intakeFuel(() -> false);
  }

  // this shit is... :oh:
  // it works
  // i guess
  public Command intakeFuel(BooleanSupplier boostActiveSupplier) {
    return runEnd(
        () -> {
          double baseSpeed = IntakeFlywheelConstants.Intake.kIntakeInSpeed;
          boolean isBoostActive = boostActiveSupplier.getAsBoolean();
          double desiredSpeed =
              isBoostActive
                  ? baseSpeed * IntakeFlywheelConstants.Control.kIntakeBoostMultiplier
                  : baseSpeed;
          commandedSpeed =
              MathUtil.clamp(
                  desiredSpeed,
                  IntakeFlywheelConstants.Intake.kMinOutput,
                  IntakeFlywheelConstants.Intake.kMaxOutput);
          boostActive = isBoostActive;
          intakeState = IntakeState.INTAKING;
          intake.setIntakeSpeed(commandedSpeed);
        },
        () -> setIntakeSpeed(0));
  }

  // boost applied to outtaking
  public Command outtakeFuel() {
    return runEnd(
        () -> {
          boostActive = false;
          setIntakeSpeed(IntakeFlywheelConstants.Intake.kIntakeOutSpeed);
          intakeState = IntakeState.OUTTAKING;
        },
        () -> setIntakeSpeed(0));
  }

  public Command stopIntake() {
    return run(() -> setIntakeSpeed(0));
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intakeSysId.quasistatic(direction);
  }

  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeSysId.dynamic(direction);
  }

  // periodic updates from the enum tree
  @Override
  public void periodic() {
    intake.updateInputs(inputs);
    Logger.processInputs("Intake/Flywheel", inputs);
  }

  @AutoLogOutput(key = "Intake/Flywheel/CommandedSpeed")
  public double getLoggedCommandedSpeed() {
    return commandedSpeed;
  }

  @AutoLogOutput(key = "Intake/Flywheel/BoostActive")
  public boolean isBoostActive() {
    return boostActive;
  }

  @AutoLogOutput(key = "Intake/Flywheel/State")
  public String getLoggedState() {
    return intakeState.toString();
  }

  private void updateStateFromSpeed(double speed) {
    if (Math.abs(speed) <= 1e-6) {
      intakeState = IntakeState.STOPPED;
    } else if (speed > 0.0) {
      intakeState = IntakeState.INTAKING;
    } else {
      intakeState = IntakeState.OUTTAKING;
    }
  }
}
