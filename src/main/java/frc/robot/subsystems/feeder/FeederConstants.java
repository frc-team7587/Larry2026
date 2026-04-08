package frc.robot.subsystems.feeder;

public class FeederConstants {
  public static final int kMotorID = 15;
  public static final double kGearRatio = 9.0;
  public static final double kNeoFreeSpeedRpm = 5676.0;
  public static final double kFeederFreeSpeedRpm = kNeoFreeSpeedRpm / kGearRatio;

  public static final double kOutSpeed = 0.65;
  public static final double kInSpeed = -0.65;
  public static final double kOutTargetRpm = kOutSpeed * kFeederFreeSpeedRpm;
  public static final double kInTargetRpm = kInSpeed * kFeederFreeSpeedRpm;

  public static final double kP = 0.001;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kMinOutput = -1.0;
  public static final double kMaxOutput = 1.0;
}
