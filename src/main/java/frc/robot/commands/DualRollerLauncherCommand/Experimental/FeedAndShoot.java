// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand.Experimental;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedAndShoot extends SequentialCommandGroup {
  private DualRollerLauncher drl;
  private ConveyorSubsystem conveyor;

  public FeedAndShoot(DualRollerLauncher drl, ConveyorSubsystem conveyor) {
    this.drl = drl;
    this.conveyor = conveyor;
    addRequirements(this.drl, this.conveyor);

    addCommands(
        new PrintCommand("Starting autonomous bullshit"),
        new SpinUpToRPM(
            drl, Constants.shooterWheelsRPMHighGoal, 0.5), // monitor window of the last .5s
        new PrintCommand("At target RPM!"),
        new ParallelRaceGroup( // run the wheel until we see a dip, then stop the conveyor (race
            // group ends all other tasks when one exits.)
            new ShootOneBall(drl, Constants.shooterWheelsRPMHighGoal),
            new AutoConveyor(conveyor, -.5, -.5)),
        new PrintCommand(
            "wheel RPM dipped outside of internal target zone, spinning back up and pausing until met condition"),
        new ParallelRaceGroup( // run the conveyor backwards until the wheel is back at speed for
            // .1s.
            new SpinUpToRPM(drl, Constants.shooterWheelsRPMHighGoal, .1),
            new AutoConveyor(conveyor, .5, .5)),
        new PrintCommand("At target for desired duration, stopping conveyors to repeat"));
  }

  @Override
  public void end(boolean interrupted) {
    this.drl.setZero();
  }
}
