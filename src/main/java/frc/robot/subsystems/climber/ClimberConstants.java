package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;

/**
 * Climber configuration, including motion parameters and
 * the climber life cycle.
 */
public class ClimberConstants {
    /**
     * State transition table for the following DFA:
     * 
     * <code>
     *   <pre>
     *                                     o
     *                                     | 
     *                                     v
     *                   +-------------->PARKED-->-----------+
     *                   |                                   |
     *                   |                                   |
     *  FULLY_RETRACTED  |         HOLD          HOLD        |
     *                   |      +--------+    +---------+    |  UNWIND
     *                   |      |        |    |         |    |
     *                   ^      ^        |    |         ^    |
     *                   |     /         v    v         \    v
     *              RETRACTING           PAUSED          EXTENDING
     *                   ^     \        /      \        /    |
     *                   |     ^       v        v       ^    v
     *                   |     |       |        |      |     |  
     *                   |     +---<---+        +-->---+     |  
     *           WIND    |       WIND            UNWIND      |  FULLY_EXTENDED
     *                   |                                   |  
     *                   |                                   |
     *                   +--------------<-RAISED-------------+
     * 
     *   </pre>
     * </code?
     */

    static final TransitionTable<State, ClimberEvent> STATE_TRANSITION_TABLE =
        TransitionTable.builder(State.IGNORE, ClimberEvent.class)
            .add(State.PARKED, ClimberEvent.UNWIND, State.EXTENDING)
            .add(State.EXTENDING, ClimberEvent.FULLY_EXTENDED, State.RAISED)
            .add(State.EXTENDING, ClimberEvent.HOLD, State.PAUSED)
            .add(State.RAISED, ClimberEvent.WIND, State.RETRACTING)
            .add(State.RETRACTING, ClimberEvent.FULLY_RETRACTED, State.PARKED)
            .add(State.RETRACTING, ClimberEvent.HOLD, State.PAUSED)
            .add(State.PAUSED, ClimberEvent.WIND, State.RETRACTING)
            .add(State.PAUSED, ClimberEvent.UNWIND, State.EXTENDING)
            .build();

    static final int kMotorId = 15;  // TODO: correct value

    static final double kUnwindSpeed = 0.4;
    static final double kWindSpeed = -0.4;

    static final double kRetractedHeight = 0.0;
    static final double kExtendedHeight = 1.0; // TODO: correct value + units

    static final double kParkedPosition = 0.0;
    // TODO: kExtendedPosition

    static final int kSmatCurrentLimit = 40;

    // TODO: configuration cribbed from the 2025 elevator. Set correct values
    static final double kP = 0.07;
    static final double kI = 0.0;
    static final double kD = 0.0;
    static final double kFF = new ElevatorFeedforward(0.1,1.44,0.6,0.05).calculate(0);
    static final double kMinOutput = -1.0;
    static final double kMaxOutput = 1.0; 
    // END Cribbed code END
}
