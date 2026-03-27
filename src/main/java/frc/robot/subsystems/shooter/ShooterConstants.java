package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public class Control {
    public static final double kStoppedSpeed = 0.0;
    public static final double kNoTargetRpm = 0.0;
    public static final double kNoStableTimestamp = -1.0;
    public static final double kTargetEpsilonRpm = 1e-6;
    // formerly 3500 default target rpm on dash
    public static final double kDashboardDefaultTargetRpm = 1500.0;
    public static final double kDashboardMaxTargetRpm = 5500.0;
  }

  public class Pivot {
    public static final int kPivotMotorID = 13;

    public static final double kPivotSpeedUp = 0.05;
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

  public class Top {
    public static final int kTopMotorID = 16;

    public static final double kOutSpeed = 0.95;
    public static final double kInSpeed = -0.95;
    public static final double kOutTargetRpm = 5000.0;
    public static final double kInTargetRpm = -5000.0;
    // og 200
    public static final double kSpeedToleranceRpm = 200.0;
    public static final double kSpeedStableTimeSec = 0.1;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double ff_kS = 0.9; // Volts
    public static final double ff_kV = 0.002135; // Volts per (radian per second)
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

    public static final double[] kDistanceMeters = {
      1.7,
    };
    public static final double[] kPivotPosition = {1};
    // public static final double[] kShooterPercentOutput = {
    //   0.58, 0.62, 0.66, 0.70, 0.74, 0.79, 0.84, 0.89, 0.94
    // };
    public static final double[] kShooterTargetRpm = {1300};
    // public static final double[] kIntakePercentOutput = {
    //   0.10, 0.11, 0.12, 0.13, 0.145, 0.16, 0.175, 0.19, 0.20
    // };
  }
}
