// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand.Experimental;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.RollingAvgDouble;
import frc.robot.utilities.UnitConversion;

public class SpinUpToRPM extends CommandBase {
  private double ShooterWheelsRPM;
  private DualRollerLauncher drl;
  private RollingAvgDouble avg;

  /* We have two constructors here to allow the use of
  both an avgWindow, defining number of items, and a
  timewindow, assuming 50Hz clock like FRC robots have
  by default.  This may change in the future. */

  public SpinUpToRPM(DualRollerLauncher drl, double wheelsRPM, int avgWindow) {
    addRequirements(drl);
    this.drl = drl;
    this.ShooterWheelsRPM = wheelsRPM;
    this.avg = new RollingAvgDouble(avgWindow);
  }

  public SpinUpToRPM(DualRollerLauncher drl, double wheelsRPM, double timeWindow) {
    addRequirements(drl);
    this.drl = drl;
    this.ShooterWheelsRPM = wheelsRPM;
    this.avg = new RollingAvgDouble(timeWindow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.avg.add(UnitConversion.RPMtoNativeUnits(ShooterWheelsRPM));
    this.drl.setShooterWheels(UnitConversion.RPMtoNativeUnits(ShooterWheelsRPM));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the buffer is full and all values are within tolerance, we're done spinning up.  checks
    // for half a second.
    if (this.avg.full()) {
      if (this.avg.lastValuesWithinTolerance(75, this.ShooterWheelsRPM)) {
        return true;
      }
    }
    return false;
  }
}
