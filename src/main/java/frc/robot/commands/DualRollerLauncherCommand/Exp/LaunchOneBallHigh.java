// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand.Exp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DualRollerLauncherCommand.NewSpinUpToRPM;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.UnitConversion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchOneBallHigh extends SequentialCommandGroup {
  private DualRollerLauncher drl;
  private ConveyorSubsystem conveyor;

  public boolean weGoodToGo() {
    return ((this.drl.rollingRPMAvg.full())
        && (this.drl.rollingRPMAvg.lastValuesWithinTolerance(
            Constants.launcherRPMTolerance, Constants.launcherRPMHighGoal)));
  }

  /** Creates a new LaunchOneBall. */
  public LaunchOneBallHigh(DualRollerLauncher drl, ConveyorSubsystem conveyor) {
    this.drl = drl;
    this.conveyor = conveyor;
    addRequirements(this.drl, this.conveyor);
    double vel = UnitConversion.RPMtoNativeUnits(Constants.launcherRPMHighGoal);

    addCommands(
        /* Before at RPM. */
        new PrintCommand("Starting spinup!"),
        new ParallelRaceGroup(
            new NewSpinUpToRPM(
                this.drl, Constants.launcherRPMHighGoal), // spins us up to our target RPM.
            new AutoConveyor(
                this.conveyor,
                -Constants.conveyorPerent,
                -Constants.feederWheelsPercent) // runs conveyor out until we're spun up.
            ),
        /* At RPM. */
        new PrintCommand("At target RPM, waiting to settle."),
        new ParallelRaceGroup(
            new RunCommand(
                () -> {
                  this.drl.setLauncherSpeedRPM(Constants.launcherRPMHighGoal);
                }), // sets it to constantly run our target RPM until then.
            new AutoConveyor(
                this.conveyor,
                -Constants.conveyorPerent,
                -Constants.feederWheelsPercent), // runs conveyors out then.
            new WaitUntilCommand(
                () -> this.weGoodToGo()) // won't exit until we're at our moving average contents.
            ),
        new PrintCommand("Settled!  Resetting moving window."),
        new InstantCommand(
            () -> {
              this.drl.rollingRPMAvg.emptyValues();
            }), // empties out rolling average so we have to wait to run again.
        /* Waiting for next shot. */
        new ParallelRaceGroup(
            new RunCommand(
                () -> {
                  this.drl.setLauncherSpeedRPM(Constants.launcherRPMHighGoal);
                }), // keeps setting the target.
            new AutoConveyor(
                this.conveyor,
                Constants.conveyorPerent,
                Constants.feederWheelsPercent), // runs conveyors in.
            new WaitUntilCommand(
                () ->
                    (!this.drl.launcherInRange(
                        Constants
                            .launcherRPMHighGoal))) // waits for the RPM dip / not in the normal
            // range to end.
            ),
        new PrintCommand("Shot Made, done!"));
  }
}
