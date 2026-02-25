package frc.robot.subsystems.conveyor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Configs.ConveyorConfig;

public class ConveyorIOSpark implements ConveyorIO {

  private final SparkMax conveyorMotor;

  public ConveyorIOSpark() {
    conveyorMotor = new SparkMax(ConveyorConstants.kMotorID, MotorType.kBrushless);

    conveyorMotor.configure(
        ConveyorConfig.conveyorMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setConveyorSpeed(double speed) {
    conveyorMotor.set(speed);
  }
}
