package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import frc.robot.Configs.ShooterConfig;
import java.util.List;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ShooterIOSpark implements ShooterIO {
  private final SparkMax topMotor;
  private final SparkMax bottomMotor;
  private final SparkMax feederMotor;
  private final SparkMax pivotMotor;

  private State m_desiredShooterState;
  private PolynomialSplineFunction m_shooterAngleCurve;
  private PolynomialSplineFunction m_shooterFlywheelCurve;

  private final RelativeEncoder pivotEncoder;
  private final SparkClosedLoopController pivotController;

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  public static final Measure<LinearVelocityUnit> ZERO_FLYWHEEL_SPEED = Units.MetersPerSecond.of(0.0);



  /** Shooter state */
  public static class State {
    public final Measure<LinearVelocityUnit> speed;
    public final Measure<AngleUnit> angle;

    // public static final State AMP_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(55.0));
    // public static final State AMP_SCORE_STATE = new State(Units.MetersPerSecond.of(+3.1), Units.Degrees.of(55.0));
    // public static final State SPEAKER_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(55.0));
    // public static final State SPEAKER_SCORE_STATE = new State(Units.MetersPerSecond.of(+15.0), Units.Degrees.of(55.0));
    // public static final State SOURCE_PREP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(55.0));
    // public static final State SOURCE_INTAKE_STATE = new State(Units.MetersPerSecond.of(-10.0), Units.Degrees.of(55.0));
    // public static final State PASSING_STATE = new State(Units.MetersPerSecond.of(+15.0), Units.Degrees.of(45.0));
    // public static final State PODIUM_SCORE_STATE = new State(Units.MetersPerSecond.of(+16.25786), Units.Degrees.of(36));
    // public static final State TEST_STOP_STATE = new State(ZERO_FLYWHEEL_SPEED, Units.Degrees.of(30.0));

    public State(Measure<LinearVelocityUnit> speed, Measure<AngleUnit> angle) {
      this.speed = speed;
      this.angle = angle;
    }
  }

  public ShooterIOSpark() {
    topMotor = new SparkMax(ShooterConstants.Top.kTopMotorID, MotorType.kBrushless);
    bottomMotor = new SparkMax(ShooterConstants.Bottom.kBottomMotorID, MotorType.kBrushless);
    feederMotor = new SparkMax(ShooterConstants.Feeder.kFeederMotorID, MotorType.kBrushless);
    pivotMotor = new SparkMax(ShooterConstants.Pivot.kPivotMotorID, MotorType.kBrushless);

    pivotEncoder = pivotMotor.getEncoder();
    pivotController = pivotMotor.getClosedLoopController();

    topMotor.configure(
        ShooterConfig.topMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    bottomMotor.configure(
        ShooterConfig.bottomMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    feederMotor.configure(
        ShooterConfig.feederMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    pivotMotor.configure(
        ShooterConfig.pivotMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void setShooterSpeed(double speed) {
    topMotor.set(speed);
  }

  @Override
  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  @Override
  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void setPivotPosition(double position) {
    pivotController.setSetpoint(position, ControlType.kPosition);
  }

  @Override
  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  /**
   * Initialize spline functions for shooter
   *
   * @param shooterMap List of distance and shooter state pairs
   */
  private void initializeShooterCurves(List<Entry<Measure<DistanceUnit>, State>> shooterMap) {
    double[] distances = new double[shooterMap.size()];
    double[] flywheelSpeeds = new double[shooterMap.size()];
    double[] angles = new double[shooterMap.size()];

    for (int i = 0; i < shooterMap.size(); i++) {
      distances[i] = shooterMap.get(i).getKey().in(Units.Meters);
      flywheelSpeeds[i] = shooterMap.get(i).getValue().speed.in(Units.MetersPerSecond);
      angles[i] = shooterMap.get(i).getValue().angle.in(Units.Radians);
    }

    m_shooterFlywheelCurve = SPLINE_INTERPOLATOR.interpolate(distances, flywheelSpeeds);
    m_shooterAngleCurve = SPLINE_INTERPOLATOR.interpolate(distances, angles);
  }

  /**
   * Set shooter to desired state
   * @param state Desired shooter state
   */
  private void setState(State state, boolean continuous) {
    // Normalize state to valid range
    m_desiredShooterState = normalizeState(state);

    // Set flywheel speed, coast to a stop if speed not desired
    if (state.speed.isNear(ZERO_FLYWHEEL_SPEED, 0.01)) {
      m_topFlywheelMotor.stopMotor();
      m_bottomFlywheelMotor.stopMotor();
    } else {
      m_topFlywheelMotor.set(m_desiredShooterState.speed.in(Units.MetersPerSecond), ControlType.kVelocity);
      m_bottomFlywheelMotor.set(m_desiredShooterState.speed.in(Units.MetersPerSecond), ControlType.kVelocity);
    }

    // Set angle
    if (continuous) m_angleMotor.set(m_desiredShooterState.angle.in(Units.Radians), ControlType.kPosition, ANGLE_FF.in(Units.Volts), ArbFFUnits.kVoltage);
    else m_angleMotor.smoothMotion(m_desiredShooterState.angle.in(Units.Radians), m_angleConstraint, motionState -> ANGLE_FF.in(Units.Volts));
  }

  /**
   * Normalize shooter state to be within valid values
   * @param state Desired state
   * @return Valid shooter state
   */
  private State normalizeState(State state) {
    Measure<Velocity<Distance>> clampedSpeed = Units.MetersPerSecond.of(MathUtil.clamp(
      state.speed.in(Units.MetersPerSecond),
      -MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond),
      +MAX_FLYWHEEL_SPEED.in(Units.MetersPerSecond)
    ));
    Measure<Angle> clampedAngle = Units.Radians.of(MathUtil.clamp(
      state.angle.in(Units.Radians),
      m_angleConfig.getLowerLimit(),
      m_angleConfig.getUpperLimit()
    ));
    return new State(clampedSpeed, clampedAngle);
  }
}
