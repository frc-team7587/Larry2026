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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  public Robot() {
    recordBuildMetadata();
    configureLogging();
    Logger.start();
    robotContainer = new RobotContainer();
  }

  private void recordBuildMetadata() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    recordGitDirtyMetadata();
  }

  private void recordGitDirtyMetadata() {
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
  }

  private void configureLogging() {
    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        configureReplayLogging();
        break;
    }
  }

  private void configureReplayLogging() {
    setUseTiming(false);
    String logPath = LogFileUtil.findReplayLog();
    Logger.setReplaySource(new WPILOGReader(logPath));
    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    recordVisualizationOutputs();
  }

  private void recordVisualizationOutputs() {
    Logger.recordOutput("RobotPose", new Pose2d());
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          new Pose3d(0, 0, 0, new Rotation3d(0.0, Math.sin(Timer.getTimestamp()) - 1.0, 0.0))
        });
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    scheduleAutonomousCommand();
  }

  private void scheduleAutonomousCommand() {
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    cancelAutonomousCommand();
  }

  private void cancelAutonomousCommand() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
