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

/**
 * The elevator life cycle, its behavior as it hoists, holds, lowers, and releases the robot.
 */
public enum State {
  /**
   * Elevator is fully retracted and not holding onto the ladder. Motor is stopped and braking is
   * OFF.
   */
  PARKED,
  /**
   * Elevator is reaching for the bar (i.e. extending).
   * Motor is unwinding.
   */
  EXTENDING,
  /**
   * Climber raised, is fully extended and ready to grab the ladder bar.
   */
  RAISED,
  /**
   * Elevator is retracting and hoisting the robot. Motor is winding.
   */
  RETRACTING,
  /**
   * Stopped, neither fully extended nor parked.
   */
  PAUSED,

  /**
   * Halted due to watchdog timeout. This is
   * an irrecoverable error.
   */
  TIMED_OUT,
  /**
   * Causes the state machine to ignore
   * the incoming event. The machine <em>NEVER</em> enters
   * this state.
   */
  IGNORE,
}
