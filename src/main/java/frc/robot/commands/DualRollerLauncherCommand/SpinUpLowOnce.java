// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.UnitConversion;

public class SpinUpLowOnce extends CommandBase {
  private DualRollerLauncher drl;
  /** Creates a new LowScore. */
  public SpinUpLowOnce(DualRollerLauncher drl) {
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
    // TODO verify again these values.
    // these were done on Saturday 2/19/2022 with Sarath and Kim.
    // they may not be 100% good, or may need to be changed for comp low
    // goal.
    this.drl.setBack(UnitConversion.RPMtoNativeUnits(800));
    this.drl.setFront(UnitConversion.RPMtoNativeUnits(1600));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.drl.frontInRange(1600) && this.drl.backInRange(800));
  }
}