package frc.robot.subsystems.shooter.ShooterFlywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Configs.ShooterConfig;

public class ShooterFlywheelIOSpark implements ShooterFlywheelIO {
  private final SparkMax topMotor;
  private final SparkMax bottomMotor;

  private final RelativeEncoder topEncoder;
  private final RelativeEncoder botEncoder;

  private final SimpleMotorFeedforward topFeedforward;
  private final SimpleMotorFeedforward bottomFeedforward;

  public ShooterFlywheelIOSpark() {

    topMotor = new SparkMax(ShooterFlywheelConstants.Top.kTopMotorID, MotorType.kBrushless);
    bottomMotor =
        new SparkMax(ShooterFlywheelConstants.Bottom.kBottomMotorID, MotorType.kBrushless);
    topFeedforward =
        new SimpleMotorFeedforward(
            ShooterFlywheelConstants.Top.ff_kS_top, ShooterFlywheelConstants.Top.ff_kV_top);
    bottomFeedforward =
        new SimpleMotorFeedforward(
            ShooterFlywheelConstants.Top.ff_kS_bot, ShooterFlywheelConstants.Top.ff_kV_bot);

    topEncoder = topMotor.getEncoder();
    botEncoder = bottomMotor.getEncoder();

    topMotor.configure(
        ShooterConfig.topMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    bottomMotor.configure(
        ShooterConfig.bottomMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterFlywheelIOInputs inputs) {
    inputs.topConnected = topMotor.getLastError() == REVLibError.kOk;
    inputs.bottomConnected = bottomMotor.getLastError() == REVLibError.kOk;
    inputs.velocityRpm = topEncoder.getVelocity();
    inputs.topAppliedVolts = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
    inputs.bottomAppliedVolts = bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage();
    inputs.topCurrentAmps = topMotor.getOutputCurrent();
    inputs.bottomCurrentAmps = bottomMotor.getOutputCurrent();

    inputs.bottomRpm = botEncoder.getVelocity();
  }

  @Override
  public void setShooterSpeed(double speed) {
    topMotor.set(speed);
    bottomMotor.set(speed);
  }

  @Override
  public void setShooterVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  public void setDuelVoltage(double top_volts, double bot_volts) {
    topMotor.setVoltage(top_volts);
    bottomMotor.setVoltage(bot_volts);
  }

  @Override
  public double getShooterVelocityRpm() {
    return topEncoder.getVelocity();
  }

  @Override
  public void setVelocity(double rpm) {
    double bottomFeedforwardVolts = bottomFeedforward.calculate(rpm);
    double topFeedforwardVolts = topFeedforward.calculate(rpm);

    topMotor
        .getClosedLoopController()
        .setSetpoint(
            rpm,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            topFeedforwardVolts,
            ArbFFUnits.kVoltage);

    bottomMotor
        .getClosedLoopController()
        .setSetpoint(
            rpm,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            bottomFeedforwardVolts,
            ArbFFUnits.kVoltage);
  }
}
