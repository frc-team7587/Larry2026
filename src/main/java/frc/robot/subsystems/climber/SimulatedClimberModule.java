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

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Module that simulates a climber.
 *
 * <p>tODO: make this correspond with reality
 *
 * @see ClimberModule
 */
public class SimulatedClimberModule extends BaseClimberModule {

  private final SparkMax motorToSimulate;
  private final SparkSim motor;
  private final SparkRelativeEncoderSim encoder;
  private final ElevatorSim elevatorSimulation;
  private double temperature;

  public SimulatedClimberModule() {
    motorToSimulate = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);
    motor = new SparkSim(motorToSimulate, ClimberConstants.kDrivebox);
    encoder = motor.getRelativeEncoderSim();
    elevatorSimulation =
        new ElevatorSim(
            ClimberConstants.kDrivebox,
            ClimberConstants.kDriveMotorReduction,
            ClimberConstants.kCarrigeMassKg,
            ClimberConstants.kDrumRadiusMeters,
            ClimberConstants.kParkedPosition,
            ClimberConstants.kExtendedElevation,
            true,
            ClimberConstants.kParkedPosition);
    temperature = 35;
  }

  @Override
  protected double doGetCurrent() {
    return motor.getMotorCurrent();
  }

  @Override
  protected double doGetOutput() {
    return motor.getAppliedOutput();
  }

  @Override
  protected double doGetTemp() {
    return temperature;
  }

  @Override
  protected double doGetVelocity() {
    return motor.getVelocity();
  }

  @Override
  protected double doGetVoltage() {
    return motor.getBusVoltage();
  }

  @Override
  protected double doGetMotorPosition() {
    return encoder.getPosition();
  }

  @Override
  void doReset() {
    motor.setPosition(0);
  }

  @Override
  protected void doSetMotorSpeed(double speed) {
    motorToSimulate.set(speed);
  }

  void setTemperature(double temperature) {
    this.temperature = temperature;
  }

  /**
   * Updates the elevator simulation, then calculates the motor RPM from its velocity and position.
   * The spool has a .75" core and its flanges are .562" apart. About 24" of .125" polycord are wond
   * on the spool when the climber is parked. If we model the wound cord as a solid cylender, we can
   * <em>roughly</em>estimage its outer diameter, which lets us estimate (again roughly) the linear
   * --> angular velocity conversion.
   */
  @Override
  public void simulationPeriodic() {
    elevatorSimulation.update(ClimberConstants.kTickTimeSeconds);

    var extension = elevatorSimulation.getPositionMeters() - ClimberConstants.kParkedPosition;
    var woundParacordLength = ClimberConstants.kParacordLengthMeters - extension;
    var totalVolume =
        ClimberConstants.kDrumVolumeCubitMeters
            + woundParacordLength * ClimberConstants.kParacordCrossSectionMetersSquared;
    var crossSection = totalVolume / ClimberConstants.kDrumWidthMeters;
    var effectiveDrumRadius = Math.sqrt(crossSection / Math.PI);
    var motorRPM = elevatorSimulation.getVelocityMetersPerSecond() / effectiveDrumRadius;
    motor.iterate(motorRPM, 12.0, ClimberConstants.kTickTimeSeconds);
  }
}
