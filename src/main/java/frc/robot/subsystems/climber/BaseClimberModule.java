package frc.robot.subsystems.climber;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

/**
 * Base climber module that implements the module life cycle and delegates
 * low-level operatioons to subclasses. There will be subclasses that
 * drive both real (i.e. physical) and simulated climbers.
 */
abstract public class BaseClimberModule implements ClimberIO {

    private static final Consumer<BaseClimberModule> DO_NOTHING =
        (m) -> {};
    private static final Consumer<BaseClimberModule> STOP_WHEN_RAISED =
            (m) -> {m.stopWhenFullyExtended();
        };
    private static final Consumer<BaseClimberModule> STOP_WHEN_PARKED =
            (m) -> {m.stopWhenFullyRetracted();
        };

    // private final SparkMax motor;
    // private final RelativeEncoder encoder;

    private State state;
    private Consumer<BaseClimberModule> periodicAction;
    private long timeInMotion;

    /**
     * Log an error message when the climber enters an unexpected
     * or unrecognized state.
     */
    private void logInvalidState() {
        Logger.recordOutput("Climber/InvalidState", state.toString());
    }

    protected BaseClimberModule() {
        timeInMotion = 0;
        reset();
    }

    /**
     * Gets motor current
     *
     * @return the current through the motor in amps.
     */
    protected abstract double doGetCurrent();

    /**
     * Gets the output of a motor
     *
     * @return the motor output in RPM
     */
    protected abstract double doGetOutput();

    /**
     * Gets the moroe temperature
     *
     * @return the motor temperature in degrees Celsius
     */
    protected abstract double doGetTemp();

    /**
     * Gets the motor velocity
     *
     * @return the algular velocity in revolutions/minute
     */
    protected  abstract double doGetVelocity();

    /**
     * Gets the voltage across a moter
     *
     * @return the voltage in volts
     */
    protected abstract double doGetVoltage();

    /**
     * Gets the position of a motor
     *
     * @return the motor position in revolutions relative to the motor's
     * orientation at power-on.
     */
    abstract protected double doGetMotorPosition();

    abstract void doReset();

    /**
     * Sets the motor speed
     *
     * @param speed desired speed. Must be in [-1.0 .. 1.0]
     */
    abstract protected void doSetMotorSpeed(double speed);

    private void enforceTimeut() {
        if (ClimberConstants.kWatchdogTimeout < ++timeInMotion) {
            receive(ClimberEvent.WATCHDOG_EXPIRED);
        }
    }

    private void stopWhenFullyExtended() {
        enforceTimeut();
        if (getPosition() >= ClimberConstants.kExtendedHeight) {
            receive(ClimberEvent.FULLY_EXTENDED);
        }
    }

    private void stopWhenFullyRetracted() {
        enforceTimeut();
        if (ClimberConstants.kParkedPosition >= getPosition()) {
            receive(ClimberEvent.FULLY_RETRACTED);
        }
    }

    @Override
    public double getCurrent() {
        return doGetCurrent();
    }

    @Override
    public double getOutput() {
        return doGetOutput();
    }

    @Override
    public double getPosition() {
        return doGetMotorPosition();
    }

    @Override
    public double getTemp() {
        return doGetTemp();
    }

    @Override
    public double getVelocity() {
        return doGetVelocity();
    }

    @Override
    public double getVoltage() {
        return doGetVoltage();
    }

    @Override
    public void periodic() {
        periodicAction.accept(this);
    }

    @Override
    public void receive(ClimberEvent event) {
        State maybeNewState =  ClimberConstants.STATE_TRANSITION_TABLE.onReceipt(state, event);
        if (State.IGNORE != maybeNewState) {
            timeInMotion = 0;
            periodicAction = DO_NOTHING;
            switch (state = maybeNewState) {
                case EXTENDING:
                periodicAction = STOP_WHEN_RAISED;
                    doSetMotorSpeed(ClimberConstants.kUnwindSpeed);
                    break;
                case IGNORE:
                   logInvalidState();
                    break;
                case PARKED:
                    doSetMotorSpeed(0);
                    break;
                case PAUSED:
                    doSetMotorSpeed(0);
                    break;
                case RAISED:
                doSetMotorSpeed(0);
                    break;
                case RETRACTING:
                    periodicAction = STOP_WHEN_PARKED;
                    doSetMotorSpeed(ClimberConstants.kWindSpeed);
                    break;
                case TIMED_OUT:
                    doSetMotorSpeed(0);
                    Logger.recordOutput("Climber/Failure", "Timeout");
                    break;
                default:
                    logInvalidState();
                    break;
            }
        }
    }

    @Override
    public void reset() {
        doReset();
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void stop() {
        receive(ClimberEvent.HOLD);
    }

    @Override
    public void wind() {
       receive(ClimberEvent.WIND);
    }

    @Override
    public void unwind() {
        receive(ClimberEvent.UNWIND);
    }
}