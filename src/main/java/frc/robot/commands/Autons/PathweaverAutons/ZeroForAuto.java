// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroForAuto extends InstantCommand {
  private DriveTrainSubsystem drive;

  public ZeroForAuto(DriveTrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.zeroHeading();
    drive.setPose(0, 0);
  }
}
