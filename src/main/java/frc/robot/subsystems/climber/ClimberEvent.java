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
 * Events to which the climber motor responds. Note that events
 * include direct commands.
 */
public enum ClimberEvent {
  /**
   * Stop immediately and hold at the current extension.
   */
  HOLD,
  /**
   * Unwind the rope, thereby raising (i.e. extending) the climber.
   */
  UNWIND,
  /**
   * Wind the rowpe, thereby lowering (i.e. retracting) the
   * climber
   */
  WIND,
  /**
   * Clmber is fully extended.
   */
  FULLY_EXTENDED,
  /**
   * Climber is fully retracted.
   */
  FULLY_RETRACTED,

  /**
   * The climber has been in motion for far too long.
   */
  WATCHDOG_EXPIRED,
}
