// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand.Exp;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DualRollerLauncherCommand.TeleopLauncherHighGoal;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BumpFeederHighGoal extends SequentialCommandGroup {
  private ConveyorSubsystem conveyor;
  private DualRollerLauncher drl;
  private PS4Controller operator;
  /** Creates a new BumpFeeder. */
  public BumpFeederHighGoal(
      DualRollerLauncher drl, ConveyorSubsystem conveyor, PS4Controller operator) {
    this.conveyor = conveyor;
    this.drl = drl;
    this.operator = operator;

    addRequirements(drl, conveyor);
    addCommands(
      new InstantCommand(() -> this.drl.pidShuffleboard()),
        new AutoConveyor(this.conveyor, -Constants.conveyorPerent, -Constants.feederWheelsPercent)
            .withTimeout(0.2),
        new ParallelCommandGroup(
            new TeleopLauncherHighGoal(this.drl, this.operator),
            new RunCommand(
                () -> {
                  if (operator.getCircleButton()) {
                    if (drl.launcherInRange(Constants.launcherRPMHighGoal)
                        || drl.launcherInRange(Constants.launcherRPMLowGoal)) {
                      this.conveyor.runConveyor(
                          -Constants.conveyorPerent, -Constants.feederWheelsPercent);
                    } else {
                      this.conveyor.runConveyor(0, 0);
                    }
                  }
                })));
  }
}
