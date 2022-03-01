// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.UnitConversion;

public class SpinUpToRPM extends CommandBase {

  private double ShooterWheelsRPM, ShooterRollersRPM;
  private DualRollerLauncher drl;
  /** Creates a new LowScore. */
  public SpinUpToRPM(DualRollerLauncher drl, double ShooterWheels, double ShooterRollers) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drl);
    this.drl = drl;
    this.ShooterWheelsRPM = ShooterWheels;
    this.ShooterRollersRPM = ShooterRollers;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.drl.setShooterRollers(UnitConversion.RPMtoNativeUnits(ShooterRollersRPM));
    this.drl.setShooterWheels(UnitConversion.RPMtoNativeUnits(ShooterWheelsRPM));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.drl.ShooterWheelsInRange(ShooterWheelsRPM)
        && this.drl.ShooterRollersInRange(ShooterRollersRPM));
  }
}
