// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Conveyor.AutoConveyor;
import frc.robot.commands.DriveTrainCommands.AutoDrive;
import frc.robot.commands.DualRollerLauncherCommand.MaintainVelocity;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallHighAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public TwoBallHighAuto(
      Intake intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AutoIntake(intake, 0.5).withTimeout(0.5), new MaintainVelocity(drl, 2600, 900)),
            new AutoConveyor(conveyor, 0.9, 0.9).withTimeout(2),
            new MaintainVelocity(drl, 0, 0), // Turns off the shooter after we shoot
            new ParallelCommandGroup(
                new AutoDrive(drive, 0.15).withTimeout(3),
                new AutoIntake(intake, 0.7).withTimeout(3)),
            new ParallelCommandGroup(
                new AutoDrive(drive, -0.15).withTimeout(3), new MaintainVelocity(drl, 2600, 900)),
            new AutoConveyor(conveyor, 0.9, 0.9)));
  }
}