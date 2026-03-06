package frc.robot.subsystems.climber;

/** 
 * Events to which the climber motor responds. Note that events
 * include direct commands.
 */
public enum ClimberEvent {
  /** 
   * Stop immediately and hold at the current extension.
   */
  HOLD,
  /** 
   * Unwind the rope, thereby raising (i.e. extending) the climber.
   */
  UNWIND,
  /** 
   * Wind the rowpe, thereby lowering (i.e. retracting) the
   * climber
   */
  WIND,
  /**
   * Clmber is fully extended.
   */
  FULLY_EXTENDED,
  /**
   * Climber is fully retracted.
   */
  FULLY_RETRACTED,

}
