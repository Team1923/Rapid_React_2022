package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DualRollerLauncherCommand.NewSpinUpToRPM;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightDeStage extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public RightDeStage(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // shooting the first ball
        new SequentialCommandGroup(
            new AutoIntake(intake, Constants.intakePercent).withTimeout(0.2),
            new ParallelRaceGroup(
                new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal).withTimeout(0.9),
                new SequentialCommandGroup(
                    new WaitCommand(0.3),
                    new AutoConveyor(
                            conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent)
                        .withTimeout(0.7))),
            new FollowPath("pathplanner/generatedJSON/RightDeStage.wpilib.json", drive)
                .setInitialHeading(true)
                .getTrajectory()
                .withTimeout(4.1),
            new InstantCommand(
                () -> {
                  drive.tankDriveVolts(0, 0);
                })));
  }
}
