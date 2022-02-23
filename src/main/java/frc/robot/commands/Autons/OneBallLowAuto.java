// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Conveyor.AutoConveyor;
import frc.robot.commands.DriveTrainCommands.AutoDrive;
import frc.robot.commands.DualRollerLauncherCommand.MaintainVelocity;
import frc.robot.commands.DualRollerLauncherCommand.SpinUpLowOnce;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallLowAuto extends SequentialCommandGroup {
  /** Creates a new OneBallAuto. */
  public OneBallLowAuto(
      Intake intake,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor,
      DualRollerLauncher drl) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AutoIntake(intake, 0.5)
                    .withTimeout(.2) // runs the drop intake command for 1/5th of a second.
                ),
            new SequentialCommandGroup(
                new SpinUpLowOnce(drl, 2700, 900), // 1600 and 800
                // this has no timeout because I defined an end condition.  We may still want to add
                // a "maintain velocity" command.
                new ParallelCommandGroup(
                    new MaintainVelocity(drl, 2700, 900).withTimeout(3),
                    new SequentialCommandGroup(
                        new AutoConveyor(conveyor, -0.5, -0.5).withTimeout(3)),
                    new RunCommand(() -> {}).withTimeout(1.5)),
                new AutoDrive(drive, .50).withTimeout(2))));
  }
}
