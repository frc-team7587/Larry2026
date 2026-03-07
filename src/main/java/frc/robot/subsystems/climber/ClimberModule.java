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

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * A concrete climber module that drives a physical climber.
 * Note that the climber motor <em>MUST</em> be controlled by
 * a SparkMax motor controller.
 *
 * @see  SimulatedClimberModule
 */
public class ClimberModule extends BaseClimberModule {


    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public ClimberModule() {
        super();
        motor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);
        encoder = motor.getEncoder();
            var config = new SparkMaxConfig()
            .smartCurrentLimit(ClimberConstants.kSmartCurrentLimit)
            .idleMode(IdleMode.kBrake);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // TODO: invoke supported counterpart instead.
            .pidf(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, 0)
            .outputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

    /**
     * Gets the current that is running throw a motor
     * controlled by a real SparkMax
     *
     * @return the current through the motor in amps
     */
    @Override
    protected double doGetCurrent() {
        return motor.getOutputCurrent();
    }

    /**
     * Gest the position of a motor controlled by a
     * real SparkMax
     *
     * @return the motor position in net rotations since power-on
     */
     @Override
    protected double doGetMotorPosition() {
        return encoder.getPosition();
    }

    /**
     * Gets the output of a motor controlled by a
     * real SparkMax
     *
     * @return the motor output in TODO: units
     */
    @Override
    protected double doGetOutput() {
        return motor.getAppliedOutput();
    }

    /**
     * Gets the temperature of a motor controoled by a
     * real SparkMax
     *
     * @return the motor temperature in degrees Celsius
     */
    @Override
    protected double doGetTemp() {
        return motor.getMotorTemperature();
    }

    /**
     * Gets the velocity of a motor controlled bhy a
     * real sparkmax
     *
     * @return the algular velocity in revolutions/minute
     */
    @Override
    protected double doGetVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Gets the voltage across a moter controlled by a real
     * SparkMax.
     *
     * @return the voltage in volts
     */@Override
    protected double doGetVoltage() {
        return motor.getBusVoltage();
    }

    @Override
    protected void doReset() {
        encoder.setPosition(ClimberConstants.kParkedPosition);
    }

    @Override
    protected void doSetMotorSpeed(double speed) {
        motor.set(speed);
    }
}
