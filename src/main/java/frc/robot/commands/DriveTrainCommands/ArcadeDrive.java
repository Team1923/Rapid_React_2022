// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDrive extends CommandBase {
  /** Creates a new DriveTest. */
  public DriveTrainSubsystem drive;

  public XboxController driver;

  public ArcadeDrive(DriveTrainSubsystem drive, XboxController driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    this.driver = driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.drive.kDrive.arcadeDrive(-1 * this.driver.getLeftY(), this.driver.getRightX());
    // this.drive.kDrive.curvatureDrive(-1 * this.driver.getLeftY(), this.driver.getRightX(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
