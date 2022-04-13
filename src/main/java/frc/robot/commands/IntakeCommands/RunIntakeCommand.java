// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.UnitConversion;

public class RunIntakeCommand extends CommandBase {

  IntakeSubsystem intake;
  PS4Controller controller;
  ConveyorSubsystem conveyor;
  /** Creates a new IntakeTest. */
  public RunIntakeCommand(
      IntakeSubsystem intake, PS4Controller controller, ConveyorSubsystem conveyor) {
    addRequirements(intake);
    this.intake = intake;
    this.controller = controller;
    this.conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getSquareButtonPressed()) {
      intake.runIntake(Constants.intakePercent);
      conveyor.runConveyorVel(UnitConversion.RPMtoNativeUnits(1000), UnitConversion.RPMtoNativeUnits(1000));

    }
    // intake out button
    if (controller.getCrossButtonPressed()) {
      intake.runIntake(-Constants.intakePercent);
      conveyor.runConveyor(Constants.conveyorPerent, Constants.feederWheelsPercent);
      // -1 * Constants.conveyorPerent
    }
    if (controller.getCircleButtonPressed()) {
      intake.runIntake(Constants.intakePercent);
    }
  }

  @Override
  public void end(boolean interrupted) {

    this.intake.runIntake(0);
    this.conveyor.runConveyor(0, 0);
  }

  // Called once the command ends or is interrupted.

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
