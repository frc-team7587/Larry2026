package frc.robot.subsystems.shooter.ShooterFlywheel;

public class ShooterFlywheelIOSim implements ShooterFlywheelIO {
  private double shooterVelocityRpm = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterFlywheelIOInputs inputs) {
    inputs.topConnected = true;
    inputs.bottomConnected = true;
    inputs.velocityRpm = shooterVelocityRpm;
    inputs.topAppliedVolts = appliedVolts;
    inputs.bottomAppliedVolts = appliedVolts;
    inputs.topCurrentAmps = 0.0;
    inputs.bottomCurrentAmps = 0.0;
  }

  @Override
  public void setShooterSpeed(double speed) {
    appliedVolts = speed * 12.0;
    shooterVelocityRpm = speed * ShooterFlywheelConstants.Top.kOutTargetRpm;
  }

  @Override
  public void setShooterVoltage(double volts) {
    appliedVolts = volts;
    shooterVelocityRpm = volts / 12.0 * ShooterFlywheelConstants.Top.kOutTargetRpm;
  }

  @Override
  public double getShooterVelocityRpm() {
    return shooterVelocityRpm;
  }

  @Override
  public void setVelocity(double rpm) {
    appliedVolts = 0.0;
    shooterVelocityRpm = rpm;
  }
}
