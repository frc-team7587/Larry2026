package frc.robot.subsystems.intake.IntakePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;



public class IntakePivotIOSim implements IntakePivotIO {
    private final DCMotor m_armGearbox = DCMotor.getNEO(2);
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.0, 4);
    private final PIDController controller = new PIDController(10, 0, 0);

    private Voltage appliedVolts = Volts.of(0.0);
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
            m_armGearbox,
            3.0, // gearing
            SingleJointedArmSim.estimateMOI(
                24, // length of arm meters
                15 // mass of arm kg
            ),
            24, // length of arm in meters
            0, // min angle in radians
            1.57, // max angle in radians
            false,
            0
        );



    @Override
    public void setPivotSpeed(double speed) {
        pivotSim.setInput(speed);
    }

    @Override
    public void setPivotPosition(double position) {
        setPivotSpeed(controller.calculate(pivotSim.getAngleRads(), position) + feedforward.calculate(pivotSim.getAngleRads(), 1));
    }

    @Override
    public double getPivotPosition() {
        return pivotSim.getAngleRads();
    }
}
