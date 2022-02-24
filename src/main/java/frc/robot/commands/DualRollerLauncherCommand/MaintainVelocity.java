// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.UnitConversion;

public class MaintainVelocity extends CommandBase {
  private DualRollerLauncher drl;
  private double ShooterWheels;
  private double ShooterRoller;

  /** Creates a new LowScore. */
  public MaintainVelocity(
      DualRollerLauncher drl, double ShooterWheelsVel, double ShooterRollerVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drl);
    this.drl = drl;
    this.ShooterWheels = ShooterWheelsVel;
    this.ShooterRoller = ShooterRollerVel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.drl.setShooterRollers(UnitConversion.RPMtoNativeUnits(this.ShooterRoller));
    this.drl.setShooterWheels(UnitConversion.RPMtoNativeUnits(this.ShooterWheels));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.drl.ShooterWheelsInRange(ShooterWheels)
        && this.drl.ShooterRollersInRange(ShooterRoller));
  }
}
