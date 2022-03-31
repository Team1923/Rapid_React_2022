package frc.robot.commands.Autons.PathweaverAutons;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
public class TwoBallAuto extends SequentialCommandGroup {
  private Trajectory get2BallTraj;
  private Trajectory backToFender;
  /** Creates a new TwoBallHighAuto. */
  public TwoBallAuto(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem driveTrain,
      ConveyorSubsystem conveyor) {

    get2BallTraj = driveTrain.generateTrajectory("pathplanner/generatedJSON/GetBall2.wpilib.json");
    backToFender =
        driveTrain.generateTrajectory("pathplanner/generatedJSON/BackToFender.wpilib.json");

    addCommands(
        new ZeroForAuto(driveTrain),
        new ParallelCommandGroup(
            new AutoIntake(intake, Constants.intakePercent),
            new SequentialCommandGroup(
                new LazyRamseteCommand(driveTrain, () -> get2BallTraj)
                    .andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new ParallelRaceGroup(new LazyRamseteCommand(driveTrain, () -> backToFender))
                    .andThen(() -> driveTrain.tankDriveVolts(0, 0))),
            new ParallelCommandGroup(
                new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal),
                new AutoConveyor(
                    conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent))));
  }
}
