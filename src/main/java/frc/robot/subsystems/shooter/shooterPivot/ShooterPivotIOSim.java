package frc.robot.subsystems.shooter.shooterPivot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterPivotIOSim implements ShooterPivotIO {

  private final DCMotor m_armGearbox = DCMotor.getNEO(1);
  private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.0, 4);
  private final PIDController controller = new PIDController(5, 0, 0);

  private Voltage appliedVolts = Volts.of(0.0);

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          m_armGearbox,
          3.0, // gearing
          SingleJointedArmSim.estimateMOI(
              10, // length of arm meters
              1 // mass of arm kg
              ),
          10, // length of arm meters
          0, // min angle in radians
          0.78539816,
          false,
          0);

  public void runAngle(double angle, double velocity) {
    sim.setInputVoltage(
        controller.calculate(sim.getAngleRads(), angle) + feedforward.calculate(angle, velocity));
  }

  @Override
  public void setPivotPosition(double position) {
    runAngle(position, 1);
  }

  @Override
  public double getPivotPosition() {
    return sim.getAngleRads();
  }
}
