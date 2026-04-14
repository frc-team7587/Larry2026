package frc.robot.subsystems.shooter.shooterFlywheel;

public class ShooterFlywheelConstants {
  public class Control {
    public static final double kStoppedSpeed = 0.0;
    public static final double kNoTargetRpm = 0.0;
    public static final double kNoStableTimestamp = -1.0;
    public static final double kTargetEpsilonRpm = 1e-6;
    // formerly 3500 default target rpm on dash
    public static final double kDashboardDefaultTargetRpm = 3500.0;
    public static final double kDashboardMaxTargetRpm = 5500.0;
  }

  public class Top {
    public static final int kTopMotorID = 16;
    public static final double kGearRatio = 3.0;

    public static final double kOutSpeed = 0.95;
    public static final double kInSpeed = -0.95;
    public static final double kOutTargetRpm = 5000.0;
    public static final double kInTargetRpm = -5000.0;
    // og 200
    public static final double kSpeedToleranceRpm = 200.0;
    public static final double kSpeedStableTimeSec = 0.1;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    // 1:1 Gear Ratio Values
    // public static final double kP = 0.00008;
    // public static final double kI = 0.0;
    // public static final double kD = 0.0;

    // public static final double ff_kS = 0.9; // Volts
    // public static final double ff_kV = 0.018715; // Volts per (radian per second)
    // 0.01868 (low by 5)
    // 0.0188 (high by 10)
    // 0.0185 (low by 12)
    // 0.0195 (high by 55)
    // 0.0160 (low by 120)
    // 0.0202 (high by 94)
    // 0.020387

    public static final double kP = 0.00008;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double ff_kS = 0.9; // Volts
    public static final double ff_kV = 0.01852; // Volts per (radian per second)
    // low by 26 (0.0185)
    // high by 63 (0.0192)
    // high by 11 (0.01875)
    // high by 11 (0.018725)
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

    // 3:1 Gear Ratio Tuning (wo/ conversion factors)
    // public static final double[] kDistanceMeters = {1.69, 1.96, 2.25, 2.45, 2.52, 2.95, 3.27};
    // public static final double[] kPivotPosition = {1.57, 2.07, 2.49, 2.4, 2.452, 3.09, 3.190};
    // // public static final double[] kShooterPercentOutput = {
    // //   0.58, 0.62, 0.66, 0.70, 0.74, 0.79, 0.84, 0.89, 0.94
    // // };
    // public static final double[] kShooterTargetRpm = {3950, 4200, 4350, 4500, 4550, 3950, 4350};
    // // public static final double[] kIntakePercentOutput = {
    // //   0.10, 0.11, 0.12, 0.13, 0.145, 0.16, 0.175, 0.19, 0.20
    // // };

    // 3:1 Gear Ratio Values (w/ conversion factors)
    // public static final double[] kDistanceMeters = {
    //   1.67, 1.78, 2.22, 2.4, 2.59, 2.85, 3.05, 3.39, 3.5
    // };
    // public static final double[] kPivotPosition = {
    //   2.19, 2.52, 2.904, 3.328, 3.40, 3.5, 3.51, 3.515, 3.58
    // };
    // public static final double[] kShooterTargetRpm = {
    //   2850, 3150, 3250, 3250, 3250, 3500, 3750, 4075, 4200
    // };
    public static final double[] kDistanceMeters = {
      1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5, 3.75, 4
    };
    public static final double[] kPivotPosition = {
      2.52, 2.85, 3.328, 3.40, 3.5, 3.51, 3.515, 3.58, 3.4, 3.43
    };
    public static final double[] kShooterTargetRpm = {
      2700, 2850, 2975, 3050, 3250, 3625, 4500, 4800, 5400, 5700
    };
  }
}
