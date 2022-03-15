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

public class TeleopRunDRLLow extends CommandBase {

  public DualRollerLauncher drl;
  private PS4Controller operator = new PS4Controller(1);

  /** Creates a new DRLTestRun. */
  public TeleopRunDRLLow(DualRollerLauncher drl) {
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
    double velWheels = UnitConversion.RPMtoNativeUnits(Constants.shooterWheelsRPMLowGoal);

    this.drl.setShooterWheels(velWheels);

    if ((drl.ShooterWheelsInRange(Constants.shooterWheelsRPMLowGoal))) {
      operator.setRumble(RumbleType.kRightRumble, 1);
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
