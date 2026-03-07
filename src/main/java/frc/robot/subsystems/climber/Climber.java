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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The climber {@link SubsystemBase Subsystem) which, among other things (DBD), provides
 * the commands that control timer behavior.
 */
public class Climber extends SubsystemBase {
    private final ClimberIO climber;

    /**
     * Creats a module that controls the specified {@code climber}
     *
     * @param climber the climber to control
     */
    public Climber(ClimberIO climber) {
        this.climber = climber;
    }

    /**
     * Extends the climber. Ignored if the climber is already extended
     *
     * @return a {@link Command} that unqinds the climber cable, which
     *         extends the climber.
     */
    public Command extend() {
        return run(
            () -> climber.unwind()
        );
    }

    /**
     * Winds the climber cable, which retracts the climber
     *
     * @return a {@link Command} that retracts the climber.
     */
    public Command retract() {
        return run(
            () -> climber.wind()
        );
    }
    /**
     * Pauses (i.e.stops) the climber in its current position, wherever it may be.
     *
     * @return a {@link Command) that pauses the timer.}
     */
    public Command pause() {
        return run(
            () -> climber.stop()
        );
    }

    /**
     * Update climber-related smart dashboard fields and performs internal climber
     * housekeeping.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", climber.getTemp());
        SmartDashboard.putNumber("Climber Voltage", climber.getVoltage());
        SmartDashboard.putNumber("Climber Output", climber.getOutput());
        SmartDashboard.putNumber("Climber Temp", climber.getTemp());
        climber.periodic();
    }

    @Override
    public void simulationPeriodic() {
        climber.simulationPeriodic();
    }
}
