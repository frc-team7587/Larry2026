package frc.robot.subsystems.conveyor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ConveyorConfig;
import java.util.List;

public class ConveyorIOSpark implements ConveyorIO {

  private final SparkMax conveyorMotor;

  public ConveyorIOSpark(List<SparkBase> motors) {
    conveyorMotor = new SparkMax(ConveyorConstants.kMotorID, MotorType.kBrushless);
    motors.add(conveyorMotor);

    conveyorMotor.configure(
        ConveyorConfig.conveyorMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setConveyorSpeed(double speed) {
    conveyorMotor.set(speed);
  }

  @Override
  public void setConveyorVoltage(double volts) {
    conveyorMotor.setVoltage(volts);
  }
}
