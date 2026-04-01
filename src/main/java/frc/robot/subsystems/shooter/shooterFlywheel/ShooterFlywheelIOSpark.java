package frc.robot.subsystems.shooter.shooterFlywheel;

import com.revrobotics.PersistMode;
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
  private final SimpleMotorFeedforward feedforward;

  public ShooterFlywheelIOSpark() {

    topMotor = new SparkMax(ShooterFlywheelConstants.Top.kTopMotorID, MotorType.kBrushless);
    bottomMotor =
        new SparkMax(ShooterFlywheelConstants.Bottom.kBottomMotorID, MotorType.kBrushless);
    feedforward =
        new SimpleMotorFeedforward(
            ShooterFlywheelConstants.Top.ff_kS, ShooterFlywheelConstants.Top.ff_kV);

    topEncoder = topMotor.getEncoder();

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
  public void setShooterSpeed(double speed) {
    topMotor.set(speed);
  }

  @Override
  public void setShooterVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public double getShooterVelocityRpm() {
    return topEncoder.getVelocity();
  }

  @Override
  public void setShooterRPM(double rpm) {
    topMotor.set(rpm);
  }

  @Override
  public void setVelocity(double rpm) {
    double angularVelocityRadPerSec = rpm * 2.0 * Math.PI / 60.0;
    double feedforwardVolts = feedforward.calculate(angularVelocityRadPerSec);
    topMotor
        .getClosedLoopController()
        .setSetpoint(
            rpm,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedforwardVolts,
            ArbFFUnits.kVoltage);
  }
}
