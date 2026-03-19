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
 * Schedules a user-provided {@link Command} to run in the future
 *
 * TODO: annotate commandHandlerCallback with @Nullable.
 */
public class DeferredCommandScheduler extends Command {
  private final Command toBeRun;
  private final long delayInTicks;
  private final WatchdogTimerSubsystem subsystem;
  private final Consumer<AutoCloseable> commandHandleCallback;

  /**
   * Constructs a {@link DeferredCommandScheduler} from the provided
   * values
   *
   * @param toBeRun the {@link Command} to run.
   * @param delayInTicks the delay in ticks. If 0 or negative, the command
   *                     runs at the next tick.
   * @param subsystem the {@link WatchdogTimerSubsystem} that will run the
   *                  command
   * @param commandHandleCallback callback that accepts the {@link AutoCloseable}
   *                              command handle.Users can cancel a command
   *                              by closing its handle. Ignored if {@code null}.
   */
  public DeferredCommandScheduler(
      Command toBeRun,
      long delayInTicks,
      WatchdogTimerSubsystem subsystem,
      Consumer<AutoCloseable> commandHandleCallback ) {
    this.toBeRun = toBeRun;
    this.delayInTicks = delayInTicks;
    this.subsystem = subsystem;
    this.commandHandleCallback  = commandHandleCallback;
    addRequirements(this.subsystem);
  }

  @Override
  public void execute() {
    var handle =
      subsystem.schedule(new CommandRunner(toBeRun), delayInTicks);
    if (commandHandleCallback != null) {
      commandHandleCallback.accept(handle);
    }
  }
}
