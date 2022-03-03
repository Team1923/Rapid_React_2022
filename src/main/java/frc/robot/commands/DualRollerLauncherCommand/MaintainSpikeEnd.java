// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand;

import frc.robot.subsystems.DualRollerLauncher;

/* This command is intended to be used as an extension of the MaintainVelocity command, specifically to end when a current behavior is detected on the motor consistent with actually launching a ball.  This lets us pause feeding in more complex autos. */
public class MaintainSpikeEnd extends MaintainVelocity {
  private double ShooterWheelsRPM, ShooterRollersRPM;
  private DualRollerLauncher drl;
  private double someValue =
      30; // 30 amps for shooter wheel stalling while not at speed.  This should be run after a
  // spin-up to ensure a standing-start isn't caught.

  public MaintainSpikeEnd(DualRollerLauncher drl, double ShooterWheels, double ShooterRollers) {
    super(drl, ShooterWheels, ShooterRollers);
    addRequirements(drl);
  }

  /* This command should end when the velocity drops and current is above some threshold (likely 20-30A?  Needs to be observed and charted.) */
  @Override
  public boolean isFinished() {
    return (!(this.drl.ShooterWheelsInRange(ShooterWheelsRPM)
            && this.drl.ShooterRollersInRange(ShooterRollersRPM))
        && this.drl.ShooterWheels.getStatorCurrent() > this.someValue);
  }
}
