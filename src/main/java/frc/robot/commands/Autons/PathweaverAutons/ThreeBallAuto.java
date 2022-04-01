package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DualRollerLauncherCommand.Exp.AutonBumpFeeder;
import frc.robot.commands.DualRollerLauncherCommand.NewSpinUpToRPM;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public ThreeBallAuto(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // shooting the first ball
        new SequentialCommandGroup(
            new AutoConveyor(conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent).withTimeout(1.5),
            // Follow the path to pick up the 2 balls
            new ParallelCommandGroup(
                new AutoIntake(intake, Constants.intakePercent),
                new SequentialCommandGroup(
                    new FollowPath("pathplanner/generatedJSON/3BallPath.wpilib.json", drive)
                        .getTrajectory(),
                    new RunCommand(
                            () -> {
                              drive.tankDriveVolts(0, 0);
                            })
                        .withTimeout(0.5))),

            // shoot the 2 balls

            new SequentialCommandGroup(
                new AutonBumpFeeder(drl, conveyor, Constants.launcherRPMHighGoal).withTimeout(.25),
                new ParallelCommandGroup(
                    new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal),
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new AutoConveyor(
                                conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent)
                            .withTimeout(0.3),
                        new WaitCommand(0.8),
                        new AutoConveyor(
                            conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent))))));
  }
}
