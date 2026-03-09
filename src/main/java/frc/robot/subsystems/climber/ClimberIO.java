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

/** Climber control API */
public interface ClimberIO {

  /**
   * @return the current flowing through the motor in amps.
   */
  double getCurrent();

  /**
   * @return the motors output, the current duty cycle as a value in [-1.0 .. _+1.0] representing a
   *     fraction of the applied voltage (e.g. .5 represetns 50% power)
   */
  double getOutput();

  /**
   * @return the motor's position in rotations relative to its starting point.
   */
  double getPosition();

  /**
   * @return the climber motor temperature in degrees Celsius
   */
  double getTemp();

  /**
   * @return the motor's velocity in RPM
   */
  double getVelocity();

  /**
   * @return the voltage across the motor in volts
   */
  double getVoltage();

  /** Housekeeping routine that must run during periodic processing. */
  void periodic();

  /**
   * Receives and processes the specified climber command.
   *
   * @param command the action that the climber should take.
   */
  void receive(ClimberEvent command);

  /**
   * Resets the climber configuration. Stops the motor, sets the state to {@link State#IDLE}, and
   * marks the current position as fully retracted. Note that the climber <em>MUST</em> be fully
   * retracted when this method runs.
   */
  void reset();

  /** Invoked periodically during simulation. Should not be invoked when the robot is running. */
  void simulationPeriodic();

  /** Stops the motor */
  void stop();

  /** Winds the climber cord on its spool, lowering the climber hook. */
  void wind();

  /** Unwinds the climber cord from the spool, raising the climber hook. */
  void unwind();
}
