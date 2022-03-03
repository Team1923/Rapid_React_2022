// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DriveTrainCommands.AutoDrive;
import frc.robot.commands.DualRollerLauncherCommand.MaintainSpikeEnd;
import frc.robot.commands.DualRollerLauncherCommand.MaintainVelocity;
import frc.robot.commands.DualRollerLauncherCommand.SpinUpToRPM;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlternativeTwoBallHighAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public AlternativeTwoBallHighAuto(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            // drop intake
            new AutoIntake(intake, 0.5).withTimeout(.2),
            new ParallelCommandGroup(
                // run intake for the duration of the auto.
                new AutoIntake(intake, 0.9),
                new SequentialCommandGroup(
                    // drive first leg and spin up, exiting when both are done.
                    new AutoDrive(drive, 0.75, 0.15).withTimeout(0.7),
                    new SpinUpToRPM(drl, 2700, 900),
                    // keep spinning and drive leg #2 with a timeout on both.
                    new ParallelCommandGroup(
                            new MaintainVelocity(drl, 2700, 900),
                            new AutoDrive(drive, -0.675, 0.275))
                        .withTimeout(2.5),

                    // spin up and keep it for 10s while agitating input, but no pause to ensure it
                    // keeps going?
                    /* TODO: Make a spin up command that launches a single ball
                    and ENDS when we dip below our target and stops the conveyor.
                     Likely to need both subsystems passed in.  This is _okay_. */

                    new SequentialCommandGroup(
                        new ParallelRaceGroup(
                            new MaintainSpikeEnd(drl, 2700, 900),
                            new AutoConveyor(conveyor, -0.9, -0.9, drl)),
                        new MaintainVelocity(drl, 2700, 900).withTimeout(.5),
                        new ParallelRaceGroup(
                            new MaintainSpikeEnd(drl, 2700, 900),
                            new AutoConveyor(conveyor, -0.9, -0.9, drl)))))));
  }
}
