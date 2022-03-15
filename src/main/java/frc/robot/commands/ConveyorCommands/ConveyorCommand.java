// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConveyorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;

public class ConveyorCommand extends CommandBase {
  /** Creates a new ConveyorTest. */
  public ConveyorSubsystem conveyor;

  private DualRollerLauncher drl;

  public ConveyorCommand(
      ConveyorSubsystem conveyor, DualRollerLauncher drl) { // ,double frontSpeed, double backSpeed
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
    this.conveyor = conveyor;
    this.drl = drl;
    // this.frontSpeed = frontSpeed;
    // this.backSpeed = backSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // checking if the shooter is range of the rpm
    if (this.drl.ShooterWheelsInRange(Constants.shooterWheelsRPMHighGoal)) {
      this.conveyor.runConveyor(
          this.conveyor.Conveyor.getDouble(0), this.conveyor.FeederWheels.getDouble(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.runConveyor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
