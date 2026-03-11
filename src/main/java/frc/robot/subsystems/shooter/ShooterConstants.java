package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public class Control {
    public static final double kStoppedSpeed = 0.0;
    public static final double kNoTargetRpm = 0.0;
    public static final double kNoStableTimestamp = -1.0;
    public static final double kTargetEpsilonRpm = 1e-6;
    public static final double kDashboardDefaultTargetRpm = 3500.0;
    public static final double kDashboardMaxTargetRpm = 5500.0;
  }

  public class Pivot {
    public static final int kPivotMotorID = 13;

    public static final double kPivotSpeedUp = 0.1;
    public static final double kPivotSpeedDown = -0.038;

    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.015;
    public static final double kFF = 0.00375;
    public static final double kGravityFFVolts = 0.3;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    public static final double kBottomPosition = 0.0;
  }

  public class Top {
    public static final int kTopMotorID = 16;

    public static final double kOutSpeed = 0.95;
    public static final double kInSpeed = -0.95;
    public static final double kOutTargetRpm = 5000.0;
    public static final double kInTargetRpm = -5000.0;
    public static final double kSpeedToleranceRpm = 200.0;
    public static final double kSpeedStableTimeSec = 0.1;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class Bottom {
    public static final int kBottomMotorID = 14;

    public static final double kOutSpeed = 0.95;
    public static final double kInSpeed = -0.95;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class AutoAim {
    // These values are meant to be tuned on-robot while watching logs.
    public static final double kVisionDistanceScale = 1.0;
    public static final double kVisionDistanceBiasMeters = 0.0;

    public static final double[] kDistanceMeters = {2.509, 4.01};
    public static final double[] kPivotPosition = {2.11, 3.0};
    // public static final double[] kShooterPercentOutput = {
    //   0.58, 0.62, 0.66, 0.70, 0.74, 0.79, 0.84, 0.89, 0.94
    // };
    public static final double[] kShooterTargetRpm = {3800, 4500};
    // public static final double[] kIntakePercentOutput = {
    //   0.10, 0.11, 0.12, 0.13, 0.145, 0.16, 0.175, 0.19, 0.20
    // };
  }
}
