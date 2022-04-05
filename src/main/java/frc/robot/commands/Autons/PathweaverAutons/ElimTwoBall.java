package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
public class ElimTwoBall extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public ElimTwoBall(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new AutoIntake(intake, Constants.intakePercent),
            new SequentialCommandGroup(

                // move forward, get ball 2
                new FollowPath("pathplanner/generatedJSON/elim2BallForward.wpilib.json", drive)
                    .setInitialHeading(true)
                    .getTrajectory()
                    .withTimeout(10),
                new InstantCommand(
                    () -> {
                      drive.tankDriveVolts(0, 0);
                    }),
                new WaitCommand(0.7),

                // move backward into fender
                new FollowPath("pathplanner/generatedJSON/elim2BallBackward.wpilib.json", drive)
                    .getTrajectory()
                    .withTimeout(10),
                new InstantCommand(
                    () -> {
                      drive.tankDriveVolts(0, 0);
                    }),

                // shoot balls
                new AutonBumpFeeder(drl, conveyor, Constants.launcherRPMHighGoal).withTimeout(.25),
                new ParallelCommandGroup(
                    new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal).withTimeout(1.8),
                    new SequentialCommandGroup(
                        new WaitCommand(0.6),
                        new AutoConveyor(
                                conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent)
                            .withTimeout(0.2),
                        new WaitCommand(0.4),
                        new AutoConveyor(
                                conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent).withTimeout(0.2),
                        new WaitCommand(0.3),
                        new AutoConveyor(
                                conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent)
                            .withTimeout(0.6))),

                // get out of the way
                new FollowPath("pathplanner/generatedJSON/elim2BallOutOfWay.wpilib.json", drive)
                    .getTrajectory()
                    .withTimeout(10),
                new InstantCommand(
                    () -> {
                      drive.tankDriveVolts(0, 0);
                    }))));
  }
}
