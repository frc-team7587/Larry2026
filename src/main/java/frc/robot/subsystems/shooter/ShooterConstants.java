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
    public static final double kSpeedToleranceRpm = 200.0;
    public static final double kSpeedStableTimeSec = 0.1;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    public static final double ff_kS = 0.9; // Volts
    public static final double ff_kV = 0.01; // Volts per (radian per second)
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
      1.8, 2.0, 2.2, 2.41, 2.6, 2.8, 3.0, 3.2, 3.5, 3.8, 4.0, 4.2, 4.4, 4.6, 4.8, 5.0
      // 5.2, 5.4, 5.6, 5.8, 6
    };
    public static final double[] kPivotPosition = {
      2.0, 2.3, 2.54, 2.65, 2.77, 2.87, 2.9, 3.01, 3.13, 3.31, 3.46, 3.6, 3.75, 3.9, 4.05, 4.2
      // 4.35, 4.5, 4.65, 4.8, 4.95, 5.1
    };
    // public static final double[] kShooterPercentOutput = {
    //   0.58, 0.62, 0.66, 0.70, 0.74, 0.79, 0.84, 0.89, 0.94
    // };
    public static final double[] kShooterTargetRpm = {
      4000, 4050, 4050, 4050, 4150, 4250, 4300, 4350, 4450, 4500, 4550, 4700, 4750, 4850, 4950, 5000
      // 4850, 4900, 4950, 5000, 5050, 5100
    };
    // public static final double[] kIntakePercentOutput = {
    //   0.10, 0.11, 0.12, 0.13, 0.145, 0.16, 0.175, 0.19, 0.20
    // };
  }
}
