package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Climber API implementation -- the heart of the matter.
 * 
 * TODO: must this be public?
 */
public class ClimberModule implements ClimberIO {

    private static final Consumer<ClimberModule> DO_NOTHING =
        (m) -> {};
    private static final Consumer<ClimberModule> STOP_WHEN_RAISED =
            (m) -> {m.stopWhenFullyExtended();
        };
    private static final Consumer<ClimberModule> STOP_WHEN_PARKED =
            (m) -> {m.stopWhenFullyRetracted();
        };

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    
    private State state;
    private Consumer<ClimberModule> periodicAction;

    /**
     * Log an error message when the climber enters an unexpected
     * or unrecognized state. 
     */
    private void logInvalidState() {
        Logger.recordOutput("Climber/InvalidState", state.toString());
    }

    public ClimberModule() {
        motor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        state = State.PARKED;
        periodicAction = DO_NOTHING;

        var config = new SparkMaxConfig()
            .smartCurrentLimit(ClimberConstants.kSmatCurrentLimit)
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: invoke supported counterpart instead.
            .pidf(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, 0)
            .outputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        reset();
    }

    private void stopWhenFullyExtended() {
        if (getPosition() >= ClimberConstants.kExtendedHeight) {
            receive(ClimberEvent.FULLY_EXTENDED);
        }
    }

    private void stopWhenFullyRetracted() {
        if (ClimberConstants.kRetractedHeight >= getPosition()) {
            receive(ClimberEvent.FULLY_RETRACTED);
        }
    }

    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public double getOutput() {
        return motor.getAppliedOutput();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public double getTemp() {
        return motor.getMotorTemperature();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public double getVoltage() {
        return motor.getBusVoltage();
    }

    @Override
    public void periodic() {
        periodicAction.accept(this);
    }

    @Override
    public void receive(ClimberEvent command) {
        State maybeNewState =  ClimberConstants.STATE_TRANSITION_TABLE.onReceipt(state, command);
        if (State.IGNORE != maybeNewState) {
            periodicAction = DO_NOTHING;
            switch (state = maybeNewState) {
                case EXTENDING:
                periodicAction = STOP_WHEN_RAISED;
                    motor.set(ClimberConstants.kUnwindSpeed);
                    break;
                case IGNORE:
                   logInvalidState();
                    break;
                case PARKED:
                motor.set(0);
                    break;
                case PAUSED:
                    motor.set(0);
                    break;
                case RAISED:
                motor.set(0);
                    break;
                case RETRACTING:
                    periodicAction = STOP_WHEN_PARKED;
                    motor.set(ClimberConstants.kWindSpeed);
                    break;
                default:
                    logInvalidState();
                    break;
            }
        }
    }

    @Override
    public void reset() {
        encoder.setPosition(ClimberConstants.kParkedPosition);
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