package frc.robot.subsystems.intake.intakeFlywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeFlywheelIOSim implements IntakeFlywheelIO {
   private final DCMotor m_motor = DCMotor.getNEO(1);

    private  final FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            m_motor,
                 0.5
            * 1 //carriage mass kg
            * 2 //drug radius in meters
            * 2 //drum radius in meters, 
             ,1), //intake gearing
        m_motor
    );

    @Override
    public void setIntakeVoltage(double voltage) {
        sim.setInputVoltage(voltage);
    }
}
