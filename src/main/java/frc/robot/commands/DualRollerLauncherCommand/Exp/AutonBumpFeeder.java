// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand.Exp;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonBumpFeeder extends SequentialCommandGroup {
  private ConveyorSubsystem conveyor;
  private DualRollerLauncher drl;

  /** Creates a new BumpFeeder. */
  public AutonBumpFeeder(DualRollerLauncher drl, ConveyorSubsystem conveyor, double rpm) {
    this.conveyor = conveyor;
    this.drl = drl;

    addRequirements(drl, conveyor);
    addCommands(
        new AutoConveyor(this.conveyor, Constants.conveyorPerent, Constants.feederWheelsPercent)
            .withTimeout(0.2),
        new RunCommand(
            () -> {
              this.drl.setLauncherSpeedRPM(rpm);
            }));
  }
}
