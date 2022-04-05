package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallStub extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public FourBallStub(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Follow the path to pick up the 2 balls
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new FollowPath("pathplanner/generatedJSON/testForward.wpilib.json", drive)
                    .setInitialHeading(true)
                    .getTrajectory()
                    .withTimeout(10),
                new InstantCommand(
                    () -> {
                      drive.tankDriveVolts(0, 0);
                    }))),

        // Follow the path to pick up the 2 balls
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new FollowPath("pathplanner/generatedJSON/testBack.wpilib.json", drive)
                    .getTrajectory(),
                new RunCommand(
                        () -> {
                          drive.tankDriveVolts(0, 0);
                        })
                    .withTimeout(0.5))));
    // shoot the 2 balls
  }
}
