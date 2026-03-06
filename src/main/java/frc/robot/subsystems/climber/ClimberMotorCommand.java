package frc.robot.subsystems.climber;

/** Actions that the climber motor can perform */
public enum ClimberMotorCommand {
  /** Stop turning. Hold in place if braking is ON. */
  STOP,

  /** Unwind the rope, thereby raising (i.e. extending) the climber. */
  UNWIND,
  /** Wind the rowpe, thereby lowering (i.e. retracting) the */
  WIND,
}
