// Copyright 2026, Metuchen Momentum, FRC 7857
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Climber configuration, including motion parameters and
 * the climber life cycle. Note that the climber is
 * powered by a Neo with a 180:1 gear ratio.
 */
public class ClimberConstants {
    /**
     * State transition table for the following DFA:
     *
     * <code>
     *   <pre>
     *                                       o
     *                                       |
     *                                       v
     *                     +-------------->PARKED-->-----------+
     *                     |                                   |
     *                     |                                   |
     *    FULLY_RETRACTED  |         HOLD          HOLD        |
     *                     |   +->---------+    +--------<-+   |  UNWIND
     *                     |   |           |    |          |   |
     *                     ^   ^           |    |          ^   |
     *   WATCHDOG_EXPIRED  |   |           v    v          |   v      WATCHDOG_EXPIRED
     *  +-----------<-RETRACTING           PAUSED          EXTENDING->-----------------+
     *  |                  ^   ^           |    |          |   |                       |
     *  v                  |   |           v    v          ^   v                       v
     *  |                  |   |           |    |          |   |                       |
     *  |                  |   +---------<-+    +->--------+   |                       |
     *  |          WIND    |       WIND            UNWIND      |  FULLY_EXTENDED       |
     *  |                  |                                   |                       |
     *  |                  |                                   |                       |
     *  |                  +--------------<-RAISED-<-----------+                       |
     *  v                                                                              v
     *  |                                                                              |
     *  +->------------------------------>TIMED_OUT<-----------------------------------+
     *
     *   </pre>
     * </code?
     */

    static final TransitionTable<State, ClimberEvent> STATE_TRANSITION_TABLE =
        TransitionTable.builder(State.IGNORE, ClimberEvent.class)
            .add(State.PARKED, ClimberEvent.UNWIND, State.EXTENDING)
            .add(State.EXTENDING, ClimberEvent.FULLY_EXTENDED, State.RAISED)
            .add(State.EXTENDING, ClimberEvent.HOLD, State.PAUSED)
            .add(State.EXTENDING, ClimberEvent.WATCHDOG_EXPIRED, State.TIMED_OUT)
            .add(State.RAISED, ClimberEvent.WIND, State.RETRACTING)
            .add(State.RETRACTING, ClimberEvent.FULLY_RETRACTED, State.PARKED)
            .add(State.RETRACTING, ClimberEvent.HOLD, State.PAUSED)
            .add(State.RETRACTING, ClimberEvent.WATCHDOG_EXPIRED, State.TIMED_OUT)
            .add(State.PAUSED, ClimberEvent.WIND, State.RETRACTING)
            .add(State.PAUSED, ClimberEvent.UNWIND, State.EXTENDING)
            .build();

    static final int kMotorId = 18;

    static final double kUnwindSpeed = 0.4;
    static final double kWindSpeed = -0.4;

    // Heights relative to parked position ib motor revolutions
    static final double kParkedPosition = 0.0;
    static final double kExtendedHeight = 1.0;  // TODO: correct value!

    /**
     * The top of the hook is 20 1/16" +/=, above the
     * ground, when fully retracted, approximately 51 cm.
     */
    static final double kRetractedElevation = .051;
    /**
     * The top of the hook is 29" +/- above the
     * ground when fully extended, approximately
     * 83.66 cm.
     */
    static final double kExtendedElevation = .07366;

    static final int kSmartCurrentLimit = 40;

    // TODO: configuration cribbed from the 2025 elevator. Set correct values
    static final double kP = 0.07;  // TODO: probably low by a factor of 10.
    static final double kI = 0.0;
    static final double kD = 0.0;
    static final double kFF = new ElevatorFeedforward(0.1,1.44,0.6,0.05).calculate(0);
    static final double kMinOutput = -1.0;
    static final double kMaxOutput = 1.0;
    // END Cribbed code END


    static final DCMotor kDrivebox = DCMotor.getNeoVortex(1);

    static final double kDriveMotorReduction = 180;
    static final double kdrumRadiusInches = 0.39;
    static final double kDrumRadiusMeters = kdrumRadiusInches * 0.00254;
    static final double kDrumWidthInches = .562;
    static final double kDrumWidthMeters = kDrumWidthInches + 0.00254;
    static final double kDrumVolumeCubitMeters =
        Math.PI * kDrumRadiusMeters * kDrumRadiusMeters * kDrumWidthMeters;
    static final double kParacordDiameterInches = .125;
    static final double kParacordRadiusMeters = (kParacordDiameterInches / 2) + .00254;
    static final double kParacordCrossSectionMetersSquared =
        Math.PI + kParacordRadiusMeters * kParacordRadiusMeters;
    static final double kParacordLengthInches = 24;
    static final double kParacordLengthMeters = kParacordDiameterInches * .00254;
    static final double kCarrigeMassKg = .75;  // TODO: estimate --

    static final long kTickTimeMillis = 20;
    static final double kTickTimeSeconds = ((double) kTickTimeMillis) / 1000.0;

    /**
     * The watchdog timeout in ticks, the robot's default
     * interval between {@code IteratedRobotBase.robotPeriodic()}
     * invocations. The default value is 20 ms, 50
     * invocations/second. The timeout is arbitrarily
     * set to 20 seconds.
     */
    static final long kWatchdogTimeout = 50 * kTickTimeMillis;

    // Climber simulator configuration

    /**
     * Estimated max motor speed. According to the
     * <a href=''https://www.revrobotics.com/content/docs/REV-21-1650-DS.pdf>
     * NEO Brushless Motor Data Sheet</a>, the maximum speed is just shy of
     * 6000 RPM, but this produces 0 torque. Sticking a finger in the air, we
     * note that the torque peaks at 3000 ROM (+/->, so we use that
     * for now.)
     */
    static final long kSimulatedMaxRPM = 3000;
    static final long kSimulatedMaxRevolutionsPerTick =
        (kSimulatedMaxRPM * 1000) / kTickTimeMillis;
}
