package frc.robot.subsystems.intake.IntakeFlywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeFlywheelIOSim implements IntakeFlywheelIO {
  private final DCMotor m_motor = DCMotor.getNEO(1);
  private double appliedVolts = 0.0;

  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              m_motor,
              0.5
                  * 1 // carriage mass kg
                  * 2 // drug radius in meters
                  * 2 // drum radius in meters,
              ,
              1), // intake gearing
          m_motor);

  @Override
  public void updateInputs(IntakeFlywheelIOInputs inputs) {
    inputs.connected = true;
    inputs.velocityRpm = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setIntakeSpeed(double speed) {
    appliedVolts = speed * 12.0;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    appliedVolts = voltage;
    sim.setInputVoltage(voltage);
  }
}
