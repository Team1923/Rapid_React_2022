// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;

public class DRLTestRun extends CommandBase {

  public DualRollerLauncher drl;
  /** Creates a new DRLTestRun. */
  public DRLTestRun(DualRollerLauncher drl) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drl);

    this.drl = drl;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.drl.setFront(0.45);
    // this.drl.setBack(0.45);

    // front 0.48 back 0.55 (Shoot other ball)
    // (kMaxRPM / 600) * (kSensorUnitsPerRotation / kGearRatio)

    this.drl.setFront();
    this.drl.setBack();

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.drl.setZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
