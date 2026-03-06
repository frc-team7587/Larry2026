package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public class Control {
    public static final double kStoppedSpeed = 0.0;
    public static final double kNoTargetRpm = 0.0;
    public static final double kNoStableTimestamp = -1.0;
    public static final double kTargetEpsilonRpm = 1e-6;
  }

  public class Pivot {
    public static final int kPivotMotorID = 13;

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
    public static final int kTopMotorID = 16;

    public static final double kOutSpeed = 0.8;
    public static final double kInSpeed = -0.8;
    public static final double kOutTargetRpm = 4200.0;
    public static final double kInTargetRpm = -4200.0;
    public static final double kSpeedToleranceRpm = 200.0;
    public static final double kSpeedStableTimeSec = 0.1;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class Bottom {
    public static final int kBottomMotorID = 14;

    public static final double kOutSpeed = 0.8;
    public static final double kInSpeed = -0.8;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class AutoAim {
    // Placeholder interpolation tables, tune on-field.
    public static final double[] kDistanceMeters = {1.5, 2.5, 3.5, 4.5, 5.5};
    public static final double[] kPivotPosition = {0.15, 0.23, 0.31, 0.39, 0.47};
    public static final double[] kShooterPercentOutput = {0.62, 0.68, 0.74, 0.80, 0.86};
    public static final double[] kShooterTargetRpm = {3200.0, 3600.0, 4000.0, 4400.0, 4800.0};
    public static final double[] kIntakePercentOutput = {0.12, 0.14, 0.16, 0.18, 0.20};
  }
}
