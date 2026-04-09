package frc.robot.subsystems.motormonitor;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TransitionTable;
import java.util.ArrayList;
import java.util.List;

/**
 * Motor monitoring systemm that compares every registered motor's temperature with a threshold and
 * alerts if the motor is too hot. Note that the threshold is set at construction
 */
public class MotorMonitor extends SubsystemBase {

  private enum AlertState {
    ALL_CLEAR, // All is good. There is nothing to do
    FAULT_DETECTED, // Report a new fault
    FAULT_REPORTED, // Waiting for reported fault to clear
    FAULT_CLEARED, // Report that the fault has been cleared
  }

  private enum Event {
    SAFE_TEMPERATURE, // Motor is at a safe temperature
    OVERHEATED, // Motor is overheated
  }

  private static final TransitionTable<AlertState, Event> TRANSITION_TABLE =
      TransitionTable.builder(AlertState.FAULT_REPORTED, Event.class)
          .add(AlertState.ALL_CLEAR, Event.SAFE_TEMPERATURE, AlertState.ALL_CLEAR)
          .add(AlertState.ALL_CLEAR, Event.OVERHEATED, AlertState.FAULT_DETECTED)
          .add(AlertState.FAULT_DETECTED, Event.SAFE_TEMPERATURE, AlertState.FAULT_CLEARED)
          .add(AlertState.FAULT_DETECTED, Event.OVERHEATED, AlertState.FAULT_REPORTED)
          .add(AlertState.FAULT_REPORTED, Event.SAFE_TEMPERATURE, AlertState.FAULT_CLEARED)
          .add(AlertState.FAULT_REPORTED, Event.OVERHEATED, AlertState.FAULT_REPORTED)
          .add(AlertState.FAULT_CLEARED, Event.SAFE_TEMPERATURE, AlertState.ALL_CLEAR)
          .add(AlertState.FAULT_CLEARED, Event.OVERHEATED, AlertState.FAULT_DETECTED)
          .build();

  private static class Motor {
    private final SparkBase motorController;
    private final Alert alert;
    private AlertState state;

    private Motor(SparkBase motorController) {
      this.motorController = motorController;
      state = AlertState.ALL_CLEAR;
      alert =
          new Alert(
              "Overheated motor, ID: " + motorController.getDeviceId(), Alert.AlertType.kError);
    }

    private void transition(Event event) {
      switch (state = TRANSITION_TABLE.onReceipt(state, event)) {
        case ALL_CLEAR:
          break;
        case FAULT_DETECTED:
          alert.set(true);
          break;
        case FAULT_REPORTED:
          break;
        case FAULT_CLEARED:
          alert.set(true);
          break;
      }
    }

    private double temperature() {
      return motorController.getMotorTemperature();
    }
  }

  private final double thresholdInCelsius;
  private final List<Motor> motors;

  public MotorMonitor(double thresholdInCelsius, List<SparkBase> motorControllers) {
    this.thresholdInCelsius = thresholdInCelsius;
    this.motors = new ArrayList<>();
    for (var motorContgroller : motorControllers) {
      motors.add(new Motor(motorContgroller));
    }
  }

  @Override
  public void periodic() {
    for (var motor : motors) {
      motor.transition(
          thresholdInCelsius < motor.temperature() ? Event.SAFE_TEMPERATURE : Event.OVERHEATED);
    }
  }
}
