package frc.robot.subsystems.shooter.ShooterPivot;

public class ShooterPivotConstants {
  public static final int kPivotMotorID = 13;

  public static final double kPivotSpeedUp = 0.08;
  public static final double kPivotSpeedDown = -0.05;

  public static final double kP = 0.26;
  public static final double kI = 0.0;
  public static final double kD = 0.01;
  public static final double kFF = 0.00375;
  public static final double kGravityFFVolts = 0.55;

  public static final double kMinOutput = -1.0;
  public static final double kMaxOutput = 1.0;

  public static final double kBottomPosition = 0.0;
}
