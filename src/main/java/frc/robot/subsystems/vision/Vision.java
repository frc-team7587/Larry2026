// Copyright 2021-2025 Team 7587 Metuchen Momentum
// https://github.com/frc-team7587
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private static class CameraLogData {
    final List<Pose3d> tagPoses = new LinkedList<>();
    final List<Pose3d> robotPoses = new LinkedList<>();
    final List<Pose3d> acceptedRobotPoses = new LinkedList<>();
    final List<Pose3d> rejectedRobotPoses = new LinkedList<>();
  }

  private static class VisionLogSummary {
    final List<Pose3d> tagPoses = new LinkedList<>();
    final List<Pose3d> robotPoses = new LinkedList<>();
    final List<Pose3d> acceptedRobotPoses = new LinkedList<>();
    final List<Pose3d> rejectedRobotPoses = new LinkedList<>();
  }

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /** Returns the latest camera-derived distance to a hub AprilTag, if available. */
  public OptionalDouble getLatestHubDistanceMeters() {
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      if (!isCameraEnabled(cameraIndex)) {
        continue;
      }
      var targetObservation = inputs[cameraIndex].latestTargetObservation;
      if (!targetObservation.hasTarget() || !isHubTag(targetObservation.tagId())) {
        continue;
      }
      if (Double.isFinite(targetObservation.distanceMeters())) {
        return OptionalDouble.of(targetObservation.distanceMeters());
      }
    }
    return OptionalDouble.empty();
  }

  @Override
  public void periodic() {
    updateCameraInputs();
    VisionLogSummary summary = new VisionLogSummary();
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      CameraLogData cameraLogData = processCamera(cameraIndex);
      logCameraData(cameraIndex, cameraLogData);
      addToSummary(summary, cameraLogData);
    }

    logSummary(summary);
  }

  private void updateCameraInputs() {
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      Logger.processInputs("Vision/Camera" + Integer.toString(cameraIndex), inputs[cameraIndex]);
    }
  }

  private CameraLogData processCamera(int cameraIndex) {
    disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
    boolean cameraEnabled = isCameraEnabled(cameraIndex);
    CameraLogData cameraLogData = new CameraLogData();

    collectTagPoses(cameraIndex, cameraLogData);
    processObservations(cameraIndex, cameraEnabled, cameraLogData);
    return cameraLogData;
  }

  private void collectTagPoses(int cameraIndex, CameraLogData cameraLogData) {
    for (int tagId : inputs[cameraIndex].tagIds) {
      var tagPose = aprilTagLayout.getTagPose(tagId);
      if (tagPose.isPresent()) {
        cameraLogData.tagPoses.add(tagPose.get());
      }
    }
  }

  private void processObservations(
      int cameraIndex, boolean cameraEnabled, CameraLogData cameraLogData) {
    for (var observation : inputs[cameraIndex].poseObservations) {
      boolean rejectPose = !cameraEnabled || shouldRejectPose(observation);

      cameraLogData.robotPoses.add(observation.pose());
      if (rejectPose) {
        cameraLogData.rejectedRobotPoses.add(observation.pose());
      } else {
        cameraLogData.acceptedRobotPoses.add(observation.pose());
      }

      if (rejectPose) {
        continue;
      }

      consumer.accept(
          observation.pose().toPose2d(),
          observation.timestamp(),
          calculateStdDevs(cameraIndex, observation));
    }
  }

  private Matrix<N3, N1> calculateStdDevs(int cameraIndex, VisionIO.PoseObservation observation) {
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = linearStdDevBaseline * stdDevFactor;
    double angularStdDev = angularStdDevBaseline * stdDevFactor;
    if (observation.type() == PoseObservationType.MEGATAG_2) {
      linearStdDev *= linearStdDevMegatag2Factor;
      angularStdDev *= angularStdDevMegatag2Factor;
    }
    if (cameraIndex < cameraStdDevFactors.length) {
      linearStdDev *= cameraStdDevFactors[cameraIndex];
      angularStdDev *= cameraStdDevFactors[cameraIndex];
    }
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  private void logCameraData(int cameraIndex, CameraLogData cameraLogData) {
    String cameraPath = "Vision/Camera" + Integer.toString(cameraIndex);
    Logger.recordOutput(
        cameraPath + "/TagPoses",
        cameraLogData.tagPoses.toArray(new Pose3d[cameraLogData.tagPoses.size()]));
    Logger.recordOutput(
        cameraPath + "/PoseObservations",
        cameraLogData.robotPoses.toArray(new Pose3d[cameraLogData.robotPoses.size()]));
    Logger.recordOutput(
        cameraPath + "/PoseObservationsAccepted",
        cameraLogData.acceptedRobotPoses.toArray(
            new Pose3d[cameraLogData.acceptedRobotPoses.size()]));
    Logger.recordOutput(
        cameraPath + "/PoseObservationsRejected",
        cameraLogData.rejectedRobotPoses.toArray(
            new Pose3d[cameraLogData.rejectedRobotPoses.size()]));
    Logger.recordOutput(cameraPath + "/Enabled", isCameraEnabled(cameraIndex));
  }

  private void addToSummary(VisionLogSummary summary, CameraLogData cameraLogData) {
    summary.tagPoses.addAll(cameraLogData.tagPoses);
    summary.robotPoses.addAll(cameraLogData.robotPoses);
    summary.acceptedRobotPoses.addAll(cameraLogData.acceptedRobotPoses);
    summary.rejectedRobotPoses.addAll(cameraLogData.rejectedRobotPoses);
  }

  private void logSummary(VisionLogSummary summary) {
    Logger.recordOutput(
        "Vision/Summary/TagPoses", summary.tagPoses.toArray(new Pose3d[summary.tagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/PoseObservations",
        summary.robotPoses.toArray(new Pose3d[summary.robotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/PoseObservationsAccepted",
        summary.acceptedRobotPoses.toArray(new Pose3d[summary.acceptedRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/PoseObservationsRejected",
        summary.rejectedRobotPoses.toArray(new Pose3d[summary.rejectedRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/HubDistanceMeters", getLatestHubDistanceMeters().orElse(-1.0));
  }

  private boolean shouldRejectPose(VisionIO.PoseObservation observation) {
    return observation.tagCount() == 0
        || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
        || Math.abs(observation.pose().getZ()) > maxZError
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth();
  }

  private static boolean isHubTag(int tagId) {
    return (tagId >= 2 && tagId <= 11) || (tagId >= 18 && tagId <= 27);
  }

  private static boolean isCameraEnabled(int cameraIndex) {
    return cameraIndex >= cameraEnabled.length || cameraEnabled[cameraIndex];
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
