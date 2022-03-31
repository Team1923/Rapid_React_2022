package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class MirroredTwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public MirroredTwoBallAuto(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem driveTrain,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new AutoIntake(intake, Constants.intakePercent),
            new SequentialCommandGroup(
                new FollowPath("paths/Testing.Path.wpilib.json", driveTrain)
                    .getTrajectory()
                    .andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new ParallelCommandGroup(
                    new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal),
                    new FollowPath("path directory here", driveTrain)
                        .getTrajectory()
                        .andThen(() -> driveTrain.tankDriveVolts(0, 0))),
                new AutoConveyor(
                    conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent))));
  }
}
