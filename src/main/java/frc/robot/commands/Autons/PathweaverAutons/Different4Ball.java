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
public class Different4Ball extends SequentialCommandGroup {
  /** Creates a new TwoBallHighAuto. */
  public Different4Ball(
      IntakeSubsystem intake,
      DualRollerLauncher drl,
      DriveTrainSubsystem drive,
      ConveyorSubsystem conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new AutoIntake(intake, Constants.intakePercent),
            //new WaitCommand(0.05),
            new SequentialCommandGroup(
                new FollowPath("pathplanner/generatedJSON/StartFourBall.wpilib.json", drive)
                    .setInitialHeading(true)
                    .getTrajectory()
                    .withTimeout(1.4),
                new InstantCommand(
                    () -> {
                      drive.tankDriveVolts(0, 0);
                    }),
                new FollowPath("pathplanner/generatedJSON/ShootFirstFourBall.wpilib.json", drive)
                    .getTrajectory()
                    .withTimeout(2.8),
                new InstantCommand(
                    () -> {
                      drive.tankDriveVolts(0, 0);
                    }),

                // shoot balls
                new AutonBumpFeeder(drl, conveyor, Constants.launcherRPMHighGoal).withTimeout(.25),
                new ParallelCommandGroup(
                        new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal).withTimeout(1),
                        new SequentialCommandGroup(
                            new WaitCommand(0.4),
                            new AutoConveyor(
                                    conveyor,
                                    Constants.conveyorPerent,
                                    Constants.feederWheelsPercent)
                                .withTimeout(0.2),
                            new WaitCommand(0.4),
                            new AutoConveyor(
                                    conveyor,
                                    Constants.conveyorPerent,
                                    Constants.feederWheelsPercent)
                                .withTimeout(0.5))
                                
                                
                                ),
                    //.withTimeout(1)
                new FollowPath("pathplanner/generatedJSON/getThreeBallFourth.wpilib.json", drive)
                    .getTrajectory(),
                    //.withTimeout(3.1),
                new InstantCommand(
                    () -> {
                      drive.tankDriveVolts(0, 0);
                    }),
                    
                new SequentialCommandGroup(
                    new AutonBumpFeeder(drl, conveyor, Constants.launcherRPMHighGoal)
                        .withTimeout(.35),
                    new ParallelCommandGroup(
                        new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal),
                        new SequentialCommandGroup(
                            new WaitCommand(0.15),
                            new AutoConveyor(
                                    conveyor,
                                    Constants.conveyorPerent,
                                    Constants.feederWheelsPercent)
                                .withTimeout(0.2),
                            new WaitCommand(0.3),
                            new AutoConveyor(
                                    conveyor,
                                    Constants.conveyorPerent,
                                    Constants.feederWheelsPercent)
                                .withTimeout(0.2),
                            new WaitCommand(0.3),
                            new AutoConveyor(
                                conveyor,
                                Constants.conveyorPerent,
                                Constants.feederWheelsPercent)))))));
  }
}
