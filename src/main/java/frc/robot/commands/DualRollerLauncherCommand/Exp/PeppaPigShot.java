package frc.robot.commands.DualRollerLauncherCommand.Exp;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DualRollerLauncherCommand.NewSpinUpToRPM;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

public class PeppaPigShot extends SequentialCommandGroup{

 private DualRollerLauncher drl;

  private ConveyorSubsystem conveyor;
  private IntakeSubsystem intake;
  private PS4Controller operator;


  public PeppaPigShot(DualRollerLauncher drl, ConveyorSubsystem conveyor, IntakeSubsystem intake, PS4Controller operator) {
    this.drl = drl;
    this.conveyor = conveyor;
    this.intake = intake;
    this.operator = operator;

    addRequirements(drl, conveyor, intake);

    addCommands(
        new ParallelCommandGroup(
            new RunCommand(() -> {this.operator.setRumble(RumbleType.kRightRumble, 1);}),
            new SequentialCommandGroup(
            new AutonBumpFeeder(drl, conveyor, Constants.launcherRPMHighGoal)
                .withTimeout(.37),
            new ParallelCommandGroup(
                new NewSpinUpToRPM(drl, Constants.launcherRPMHighGoal),
                new AutoIntake(intake, Constants.intakePercent),
                new SequentialCommandGroup(
                    new WaitCommand(0.4),
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
                    new WaitCommand(0.2),
                    new AutoConveyor(
                            conveyor,
                            Constants.conveyorPerent,
                            Constants.feederWheelsPercent)
                        .withTimeout(0.2),
                    new WaitCommand(0.3),
                    new AutoConveyor(
                        conveyor,
                        Constants.conveyorPerent,
                        Constants.feederWheelsPercent))))

        )   
        

    );
    
  }

  @Override
  public void end(boolean interrupt) {
    this.drl.setZero();
    this.conveyor.runConveyor(0, 0);
    this.intake.runIntake(0);
    this.operator.setRumble(RumbleType.kRightRumble, 0);
  }
}