// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrainCommands.AutoDrive;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public FourBallAuto(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AutoIntake(intake, 0.9),
                new SequentialCommandGroup(
                    new AutoDrive(drive, 0.1, 0.563).withTimeout(0.5), // turning in place
                    new AutoDrive(drive, 0.7, 0).withTimeout(3.1),
                    new AutoDrive(drive, -0.7, 0).withTimeout(3),
                    new AutoDrive(drive, -0.1, -0.56).withTimeout(0.3)))));
    // new AutoDrive(drive, -0.6, 0).withTimeout(1)
    // new NewSpinUpToRPM(drl, 4050)

  }
}