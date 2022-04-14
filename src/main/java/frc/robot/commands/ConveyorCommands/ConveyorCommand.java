// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConveyorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.UnitConversion;

public class ConveyorCommand extends CommandBase {

  private ConveyorSubsystem conveyor;
  private DualRollerLauncher drl;

  public ConveyorCommand(
      ConveyorSubsystem conveyor, DualRollerLauncher drl) { // ,double frontSpeed, double backSpeed
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyor);
    this.conveyor = conveyor;
    this.drl = drl;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /* The intent of this logic is to check if we're in the boolean range of either
   the high or low goal in an instant, and if so run the conveyor to feed it,
   *otherwise* to feed it out.

  With swapping invert states this may not be perfect and may need some work.
  */
  @Override
  public void execute() {


    if (drl.launcherInRange(Constants.launcherRPMHighGoal)) {
      // || drl.launcherInRange(Constants.launcherRPMLowGoal)) {
      this.conveyor.runConveyorVel(
          UnitConversion.RPMtoNativeUnits(1000), UnitConversion.RPMtoNativeUnits(1000));
    } else {
      this.conveyor.runConveyor(0, 0);
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
