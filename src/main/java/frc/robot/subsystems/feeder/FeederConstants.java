package frc.robot.subsystems.feeder;

public class FeederConstants {
  public static final int kMotorID = 15;
  public static final double kGearRatio = 5.0;

  public static final double kOutSpeed = 0.65;
  public static final double kInSpeed = -0.65;
  public static final double kOutTargetRpm = 1000;
  public static final double kInTargetRpm = 1000;
  public static final double kIdleTargetRpm = 200;

  public static final double kP = 0.001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kS = 0.7175;
  public static final double kV = 0.00995;

  public static final double kMinOutput = -1.0;
  public static final double kMaxOutput = 1.0;
}
