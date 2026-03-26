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
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.watchdogtimer.CommandRunner;
import frc.robot.subsystems.watchdogtimer.WatchdogTimerSubsystem;
import java.util.function.Consumer;

/**
 * Schedules a {@link Runnable} to run after a delay
 *
 * <p>Note: while the {@code RunOnTimeout|} command uses the {@link WatchdogTimerSubsystem}, oit
 * <em>DOES NOT</em> require exclusive access to it. The {@link WatchdogTimerSubsystem} is designed
 * to manage an arbitrary number of pending {@code RunOnTimeout} instances, so a resource conflich
 * simply cannot occur. In fact, they are encouraged. Therefore, the {@code RunOnTimeout} command
 * deliberately <em>DOES NOT</em> incorporate the {@link WatchdogTimerSubsystem} in its requirements
 * list.
 *
 * <p>The enqueued command <em>MAY</em> have requirements that will apply <em>when it runs</em>.
 * Since its requirements take effect <em>if the command runs</em> (which it may not), they are
 * irrelvant while it is pending. Hence, we do not incorporate then into its conbtaining {@cide
 * RunOnTimeout}.
 */
public class RunOnTimeout extends Command {
  private final Runnable action;
  private final long delayInTicks;
  private static WatchdogTimerSubsystem subsystem;
  private Consumer<AutoCloseable> commandHandleCallback;

  /**
   * Constructor
   *
   * @param action action to run when the timeout expires
   * @param delayInTicks delay in arbitrary ticks. THe default tick length is 20 milliseconds. This
   *     can be changed.
   * @param commandHandleCallback invoked on execution so that the caller may receive the command
   *     handle.
   */
  private RunOnTimeout(
      Runnable action, long delayInTicks, Consumer<AutoCloseable> commandHandleCallback) {
    this.action = action;
    this.delayInTicks = delayInTicks;
    this.commandHandleCallback = commandHandleCallback;
  }

  /**
   * Sets the {@link WatchtodTimerSubsystem} that runs the commands on timeout. The RobotContainer
   * <em>MUST</em> invoke this method immediatel upon construction.
   *
   * @param watchdogTimerSubsystem the subsystem, as described above. Cannot be {@ocde null}
   */
  public static void setSubsystem(WatchdogTimerSubsystem watchdogTimerSubsystem) {
    subsystem = watchdogTimerSubsystem;
  }

  /**
   * Creates a {@link RunOnTimeout} command that schedules a {@link Runnable} action to run when a
   * specified timeout expires.
   *
   * @param action action to run with the timeout expires
   * @param delayInTicks timeout in ticks. If the delay is less than or equal to 0, the action will
   *     run on the first tick after it is scheduled.
   * @param commandHandleCallback invoked to pass the command handle to the caller. Ignored if
   *     {@code null}, otherwise invoked when the returned command runs.
   * @return the newly created {@link RunOnTimeout} command.
   */
  public static RunOnTimeout of(
      Runnable action, long delayInTicks, Consumer<AutoCloseable> commandHandleCallback) {
    return new RunOnTimeout(action, delayInTicks, commandHandleCallback);
  }

  /**
   * Creates a {@link RunOnTimeout} that schedules a {@link Command} to be performed when a
   * specified timeout expires.
   *
   * @param command the {@link Command} to be performed when the specified timeout expires. Note
   *     that the returned {@link RunOnTimeout} command in corporates {@code command}'s
   *     requirements.'
   * @param delayInTicks timeout in ticks. If the delay is less than or equal to 0, the action will
   *     run on the first tick after it is scheduled.
   * @param commandHandlerCallback invoked to pass the command handle to the caller. Ignored if
   *     {@code null}, otherwise invoked when the returned command runs.
   * @return the newly created {@link RunOnTimeout} command.
   */
  public static RunOnTimeout of(
      Command command, long delayInTicks, Consumer<AutoCloseable> commandHandlerCallback) {
    return new RunOnTimeout(new CommandRunner(command), delayInTicks, commandHandlerCallback);
  }

  /**
   * Command payload, which schedules this instance's {@link #action} to run when the timeout
   * expires.
   */
  @Override
  public void execute() {
    var handle = subsystem.schedule(action, delayInTicks);
    if (commandHandleCallback != null) {
      commandHandleCallback.accept(handle);
    }
  }
}
