// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConveyorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class AutoConveyor extends CommandBase {
  public ConveyorSubsystem conveyor;
  private double belts, wheels;

  public AutoConveyor(
      ConveyorSubsystem conveyor,
      double belts,
      double wheels) { // ,double frontSpeed, double backSpeed
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
    this.conveyor = conveyor;

    this.belts = belts;
    this.wheels = wheels;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.conveyor.runConveyor(belts, wheels);
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
