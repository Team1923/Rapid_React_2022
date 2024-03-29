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
public class OneBallHighAuto extends SequentialCommandGroup {
  /** Creates a new OneBallAuto. */
  public OneBallHighAuto(
      IntakeSubsystem intake,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor,
      DualRollerLauncher drl) {
    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(new AutoIntake(intake, 0.5).withTimeout(.2)),
            new SequentialCommandGroup(
                new NewSpinUpToRPM(drl, 4050),
                new ParallelCommandGroup(
                    new NewSpinUpToRPM(drl, 4050).withTimeout(3),
                    new SequentialCommandGroup(
                        new AutoConveyor(conveyor, -0.5, -0.5).withTimeout(3)),
                    new RunCommand(() -> {}).withTimeout(1.5)),
                new AutoDrive(drive, .50, 0).withTimeout(2.5))));
  }
}
