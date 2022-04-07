package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DualRollerLauncherCommand.Exp.AutonBumpFeeder;
import frc.robot.commands.DualRollerLauncherCommand.NewSpinUpToRPM;
import frc.robot.commands.DualRollerLauncherCommand.TeleopLauncherHighGoal;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrollTwoBall extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public TrollTwoBall(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup( 
            new AutoIntake(intake, Constants.intakePercent).withTimeout(8.5),
            new SequentialCommandGroup(
                new FollowPath("pathplanner/generatedJSON/2BallMirrored.wpilib.json", drive)
                    .setInitialHeading(true)
                    .getTrajectory()
                    .withTimeout(4.1),
                new InstantCommand(
                        () -> {
                          drive.tankDriveVolts(0, 0);
                        }),
                new AutonBumpFeeder(drl, conveyor, Constants.launcherRPMHighGoal).withTimeout(.25),
                new ParallelRaceGroup(
                    new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal).withTimeout(1),
                    new SequentialCommandGroup(
                        new WaitCommand(0.15),
                        new AutoConveyor(
                                conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent)
                            .withTimeout(0.2),
                        new WaitCommand(0.25),
                        new AutoConveyor(
                            conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent))),
                //trolling heh
                new FollowPath("pathplanner/generatedJSON/TrollPath.wpilib.json", drive).getTrajectory().withTimeout(4)
                            
                            
                            
                            )),
                //spit-out
                new AutoIntake(intake, -Constants.intakePercent)            
                );
  }
}
