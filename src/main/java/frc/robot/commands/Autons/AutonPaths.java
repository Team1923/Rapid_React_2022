// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveTrainCommands.DriveTest;
import frc.robot.commands.DualRollerLauncherCommand.DRLTestRun;
import frc.robot.commands.Intake.IntakeTest;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DualRollerLauncher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonPaths extends SequentialCommandGroup {
  /** Creates a new AutonPaths. */
  Trajectory auton = PathPlanner.loadPath("2 ball Auto", 1, 0.2);

  public AutonPaths() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(
    //     new ParallelCommandGroup(
    //         new IntakeTest(new Intake(), operator), new DRLTestRun(new DualRollerLauncher())),
    //     new DriveTest(new DriveTrainSubsystem(), null));
  }
}
