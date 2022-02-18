// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

  Intake intake;
  PS4Controller controller;
  /** Creates a new IntakeTest. */
  public IntakeCommand(Intake intake, PS4Controller controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getSquareButtonPressed()) {
      intake.runIntake(-1 * this.intake.intakeValue.getDouble(0));
    }
    if (controller.getCrossButtonPressed()) {
      intake.runIntake(this.intake.intakeValue.getDouble(0));
    }
  }

  @Override
  public void end(boolean interrupted) {

    this.intake.runIntake(0);
  }

  // Called once the command ends or is interrupted.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
