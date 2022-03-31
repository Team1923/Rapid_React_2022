// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import java.util.function.Supplier;

/* Major parts of the supplier / subscriber implementation were  borrowed from 4277.  Thank you!

https://github.com/FRC-4277/2020InfiniteRecharge/blob/378095f4f194974fdc7e5202eb25e17ac806d263/src/main/java/frc/robot/commands/autonomous/LazyRamseteCommand.java

*/

public class LazyRamseteCommand extends CommandBase {
  private DriveTrainSubsystem driveTrain;
  private Supplier<Trajectory> trajectorySupplier;
  private RamseteCommand ramseteCommand;

  public LazyRamseteCommand(DriveTrainSubsystem drive, Supplier<Trajectory> trajectorySupplier) {
    this.driveTrain = drive;
    this.trajectorySupplier = trajectorySupplier;
    addRequirements(this.driveTrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (ramseteCommand == null && trajectorySupplier.get() != null) {
      ramseteCommand =
          new RamseteCommand(
              trajectorySupplier.get(),
              driveTrain::getPose,
              new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
              Constants.m_feedforward,
              Constants.kDriveKinematics,
              driveTrain::getWheelSpeeds,
              driveTrain.getLeftPidController(),
              driveTrain.getRightPidController(),
              driveTrain::tankDriveVolts,
              driveTrain);
      ramseteCommand.initialize();
      return;
    }
    if (ramseteCommand != null) {
      ramseteCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (ramseteCommand != null) {
      ramseteCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return ramseteCommand != null && ramseteCommand.isFinished();
  }
}
