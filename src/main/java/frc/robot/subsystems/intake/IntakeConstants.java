package frc.robot.subsystems.intake;

public class IntakeConstants {
  public class Pivot {
    public static final int kLeaderID = 11;
    public static final int kFollowerID = 10;

    public static final double kPivotSpeedUp = 0.12;
    public static final double kPivotSpeedDown = -0.12;

    public static final double kP = 0.07;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.00375;

    public static final double kMinOutput = -0.4;
    public static final double kMaxOutput = 0.4;
  }

  public class Intake {
    public static final int kIntakeMotorID = 12;

    public static final double kIntakeOutSpeed = -0.4;
    public static final double kIntakeInSpeed = 0.4;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }
}
