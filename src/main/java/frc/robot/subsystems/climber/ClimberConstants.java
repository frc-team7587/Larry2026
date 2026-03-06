package frc.robot.subsystems.climber;

public class ClimberConstants {
  
  public static final int kMotorId = 17;

  public static final double kGearRatio = 180.0;
  public static final double kNeoFreeSpeedRpm = 5676.0;
  public static final double kClimberFreeSpeedRpm = kNeoFreeSpeedRpm / kGearRatio;

  public static final double kClimbUpSpeed = 0.8;
  public static final double kClimbDownSpeed = -0.5;

  public static final double kP = 0.6;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;

  public static final double kMinOutput = -1.0;
  public static final double kMaxOutput = 1.0;
}
