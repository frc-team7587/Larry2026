package frc.robot.subsystems.intake.IntakeFlywheel;

public class IntakeFlywheelConstants {
  public class Intake {
    public static final int kLeaderMotorID = 12;
    public static final int kFollowerMotorID = 19;

    // public static final double kIntakeOutSpeed = -0.31;
    // public static final double kIntakeInSpeed = 0.31;
    public static final double kIntakeOutSpeed = -0.31;
    public static final double kIntakeInSpeed = 0.31;

    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
  }

  public class Control {
    // boost on intake speed mmultiplier (1.25 is 25% boost  for ex.)
    public static final double kIntakeBoostMultiplier = 1.6;
  }
}
