package frc.robot.subsystems.shooter;

public interface ShooterIO {
  public void setShooterSpeed(double speed);

  public void setFeederSpeed(double speed);

  public void setPivotSpeed(double speed);

  public void setPivotPosition(double position);

  public double getPivotPosition();
}
