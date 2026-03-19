
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

package frc.robot.subsystems.watchdogtimer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.metuchenmomentum.util.WatchdogTimer;;

/**
 * Creates and drives a {@link WatchdogTimer}, a class that invokes
 * invokes {@link Runnable} implementations after a fixed delay.
 * The subsystem passes timer ticks to the {@link WatchdogTimer}
 * so that it will invoke enqueued {@link Runnable} instances when
 * their associated timeouts expire.
 */
public final class WatchdogTimerSubsystem extends SubsystemBase {

  private final WatchdogTimer watchdogTimer;

  /**
   * Constructor, instantiates a {@link WatchdogTimerSubsystem} with
   * an empty queue.
   */
  public WatchdogTimerSubsystem() {
    watchdogTimer = new WatchdogTimer();
  }

  /**
   * Advances the watchdog timer by 1 tick. Invoked during
   * robot operation.
   */
  @Override
  public void periodic() {
    watchdogTimer.periodic();
  }

  /**
   * Advances the watchdog timer by 1 tick. Invoked during
   * robot simulation
   */
  @Override
  public void simulationPeriodic() {
    watchdogTimer.periodic();
  }

  /**
   * Schedule the specified action to run after the specified deadline.
   *
   * @param action the action to run. Cannot be {@code null}. This is not
   *               checked. Note that the action will remain scheduled until
   *               its deadline expires, even if the user cancels it. However,
   *               a scheduled action will not run.
   * @param delayInTicks timeout in ticks relative to the current tick count.
   *                     The value <em>SHOULD</em> be strictly positive. If
   *                     it is zero or negative, the action will run at the
   *                     next tick.
   * @return an {@link AutoCloseable} action handle. Users can invoke the handle's
   *         {@link AutoCloseable#close() close()} method to cancel the action.
   *         Note that cancelling the action <em>DOES NOT</em> will remain
   *         scheduled, but its action will not run when its deadline expires.
   */
  public AutoCloseable schedule(Runnable action, long delayInTicks) {
    return watchdogTimer.schedule(action, delayInTicks);
  }

}
