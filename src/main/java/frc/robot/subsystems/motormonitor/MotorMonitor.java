package frc.robot.subsystems.motormonitor;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * Motor monitoring systemm that compares every registered motor's temperature with a threshold and
 * alerts if the motor is too hot. Note that the threshold is set at construction
 */
public class MotorMonitor extends SubsystemBase {

  private final double thresholdInCelsius;
  private final List<SparkBase> motors;

  public MotorMonitor(double thresholdInCelsius, List<SparkBase> motors) {
    this.thresholdInCelsius = thresholdInCelsius;
    this.motors = new ArrayList<>(motors);
  }

  @Override
  public void periodic() {
    for (var motor : motors) {
      if (thresholdInCelsius <= motor.getMotorTemperature()) {
        // Note: since we need to include the motor ID in the
        //       alert message, we need to create the alert
        //       when we detect the error. Otherwise, we
        //       because the alert message is set at
        //       construction, we could not include
        //       the motor ID.
        try (Alert alert =
            new Alert("Overheated motor, ID: " + motor.getDeviceId(), Alert.AlertType.kError); ) {
          alert.set(true);
        } finally {
          // Auto-close the alert.
        }
      }
    }
  }
}
