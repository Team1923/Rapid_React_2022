// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.UnitConversion;

public class TeleopLauncherHighGoal extends CommandBase {

  public DualRollerLauncher drl;
  private PS4Controller operator;

  /** Creates a new DRLTestRun. */
  public TeleopLauncherHighGoal(DualRollerLauncher drl, PS4Controller operator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drl);
    this.operator = operator;
    this.drl = drl;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.drl.pidShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double vel = UnitConversion.RPMtoNativeUnits(this.drl.rpm.getDouble(0));
    this.drl.setLauncherSpeedRPM(vel);

    if (this.drl.launcherInRange(Constants.launcherRPMHighGoal)) {
      this.operator.setRumble(RumbleType.kRightRumble, 1);
    } else {
      this.operator.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.drl.setZero();
    operator.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
