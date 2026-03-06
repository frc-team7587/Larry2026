package frc.robot.subsystems.climber;

/**
 * The elevator life cycle, its behavior as it hoists, holds, lowers, and releases the robot.
 *
 * <pre>
 * <code>
 *
 *             +------------>IDLE-->-------+
 *             |                           |
 *  STOP       ^                           |   RAISE
 *             |                           v
 *           PARKING                     REACHING
 *  LOWER      |                           |
 *             ^                           |    STOP
 *             |                           |
 *          RELEASED                     POISED
 *             |                           |
 *  STOP       ^                           |     LOWER
 *             |                           v
 *          LOWERING                    HOISTING
 *             |                           |
 *  RAISE      ^                           |    STOP
 *             |                           |
 *             +----------->LOCKED-->----- +
 * </code>
 * </pre>
 */
public enum State {
  /**
   * Elevator is fully retracted and not holding onto the ladder. Motor is stopped and braking is
   * OFF.
   */
  IDLE,
  /**
   * Elevator is reaching for the bar (i.e. extending) and so it can grab it and hoist the robot.
   * Motor is unwinding and braking is OFF.
   */
  REACHING,
  /** Climber is fully extended and ready to grab the ladder bar. Moter os off and braking is ON. */
  POISED,
  /** Elevator is retracting and hoisting the robot. Motor is winding and braking is ON. */
  HOISTING,
  /**
   * Elevator is fully retracted and locked, holding the robot above the field. Motor is stopped and
   * braking is ON.
   */
  LOCKED,
  /**
   * Climber is fully lowering the robot to the field the field.Motor is unwinding and braking is
   * OFF.
   */
  LOWERING,
  /** The climber has lowered the robot to the ground. Motor is stopped and braking is off. */
  RELEASED,
  /** Climber is retracting to its rest position.. Motor is winjding and braking is OFF. */
  PARKING,
  /** A fault has occurred. Motor is stopped and braking is ON. */
  FAULT,
}
