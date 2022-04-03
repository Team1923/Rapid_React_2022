// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import java.io.IOException;
import java.nio.file.Path;

public class FollowPath {
  /** Creates a new FollowPath. */
  private String path;

  private DriveTrainSubsystem driveTrain;
  private Trajectory trajectory = new Trajectory();
  private boolean setInitialHeading = false;

  public FollowPath(String path, DriveTrainSubsystem driveTrain) {
    this.path = path;
    this.driveTrain = driveTrain;
  }

  public FollowPath setInitialHeading(boolean setHeading) {
    this.setInitialHeading = setHeading;
    return this;
  }

  public Pose2d getInitialPose() {
    return trajectory.getInitialPose();
  }

  public RamseteCommand getTrajectory() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("not opening", ex.getStackTrace());
    }

    System.out.println(
        "[path] running path for seconds (count): " + (trajectory.getTotalTimeSeconds()));

    if (this.setInitialHeading) {
      driveTrain.setPose(trajectory.getInitialPose());
      System.out.println("Start of path, reset pose.");
    }

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            driveTrain.getFeedForward(),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            driveTrain.getLeftPidController(),
            driveTrain.getRightPidController(),
            driveTrain::tankDriveVolts,
            driveTrain);

    return ramseteCommand;
  }
}
