package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public class Pivot {
    public static final int kPivotMotorID = 11;

    public static final double kPivotSpeedUp = 0.1;
    public static final double kPivotSpeedDown = -0.1;

    public static final double kP = 0.07;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.00375;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class Top {
    public static final int kTopMotorID = 12;

    public static final double kOutSpeed = 0.8;
    public static final double kInSpeed = -0.8;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class Bottom {
    public static final int kBottomMotorID = 12;

    public static final double kOutSpeed = 0.8;
    public static final double kInSpeed = -0.8;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class Feeder {
    public static final int kFeederMotorID = 12;

    public static final double kOutSpeed = 0.8;
    public static final double kInSpeed = -0.8;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }
}
