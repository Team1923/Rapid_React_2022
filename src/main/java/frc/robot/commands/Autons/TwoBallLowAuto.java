// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DriveTrainCommands.AutoDrive;
import frc.robot.commands.DualRollerLauncherCommand.NewSpinUpToRPM;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallLowAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public TwoBallLowAuto(
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
                    new AutoDrive(drive, 0.6, 0.15).withTimeout(0.6),
                    new NewSpinUpToRPM(drl, 4050),
                    new RunCommand(() -> {}).withTimeout(0.3),
                    // keep spinning and drive leg #2 with a timeout on both.
                    new ParallelCommandGroup(
                            new NewSpinUpToRPM(drl, 4050), new AutoDrive(drive, -0.675, 0.23))
                        .withTimeout(2),
                    new ParallelCommandGroup(
                        new NewSpinUpToRPM(drl, 4050).withTimeout(10),
                        new SequentialCommandGroup(
                            new AutoDrive(drive, -0.5, 0).withTimeout(0.9),
                            new AutoConveyor(conveyor, -0.9, -0.9).withTimeout(0.3),
                            new RunCommand(() -> {}).withTimeout(0.5),
                            new AutoConveyor(conveyor, -0.9, -0.9).withTimeout(0.3),
                            new RunCommand(() -> {}).withTimeout(0.5),
                            new AutoConveyor(conveyor, -0.9, -0.9).withTimeout(0.3),
                            new RunCommand(() -> {}).withTimeout(0.5),
                            new AutoConveyor(conveyor, -0.9, -0.9).withTimeout(0.3),
                            new RunCommand(() -> {}).withTimeout(0.5)))))));
  }
}
