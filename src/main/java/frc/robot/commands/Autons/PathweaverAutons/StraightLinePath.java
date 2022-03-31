package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightLinePath extends SequentialCommandGroup {
  private Trajectory testPath;
  /** Creates a new TwoBallHighAuto. */
  public StraightLinePath(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem driveTrain,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.testPath = driveTrain.generateTrajectory("paths/Testing.Path.wpilib.json");

    addCommands(
        new SequentialCommandGroup(
            new AutoIntake(intake, Constants.intakePercent),
            new ParallelCommandGroup(
                new LazyRamseteCommand(driveTrain, () -> testPath)
                    .andThen(() -> driveTrain.tankDriveVolts(0, 0)))));
  }
}
