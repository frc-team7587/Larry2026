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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.InternalButton;

/** A runnable that enqueues a {@link Command} for execution */
public class CommandRunner implements Runnable {

  private final Command command;

  /**
   * Constructs an instance from the specified values.
   *
   * @param command the command to invoke from {@link #run()}
   */
  public CommandRunner(Command command) {
    this.command = command;
  }

  /** Runs the {@link Command} that is bound to this instance. */
  @Override
  public void run() {
    InternalButton button = new InternalButton();
    button.onTrue(command);
    button.setPressed(true);
  }
}
