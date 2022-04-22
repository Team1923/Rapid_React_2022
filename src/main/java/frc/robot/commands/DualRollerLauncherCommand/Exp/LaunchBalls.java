// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DualRollerLauncherCommand.Exp;

import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ConveyorCommands.AutoConveyor;
import frc.robot.commands.DualRollerLauncherCommand.TestRPMSpinLogic;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

public class LaunchBalls extends SequentialCommandGroup {
  /** Creates a new LaunchBalls. */
  private DualRollerLauncher drl;

  private ConveyorSubsystem conveyor;
  private IntakeSubsystem intake;
  private PS4Controller operator;
  private Timer timer;

  public boolean weGoodToGo() {
    return ((this.drl.rollingRPMAvg.full())
        && (this.drl.rollingRPMAvg.lastValuesWithinTolerance(
            Constants.launcherRPMTolerance, Constants.launcherRPMHighGoal)));
  }

  public LaunchBalls(DualRollerLauncher drl, ConveyorSubsystem conveyor, IntakeSubsystem intake, PS4Controller operator) {
    this.drl = drl;
    this.conveyor = conveyor;
    this.intake = intake;
    this.timer = new Timer();
    this.operator = operator;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drl, conveyor, intake);

    addCommands(
        new InstantCommand(
            () -> {
              this.timer.reset();
              this.timer.start();
            }),


        new ParallelCommandGroup(
            new AutoConveyor(
                    this.conveyor, -Constants.conveyorPerent, -Constants.feederWheelsPercent)
                .withTimeout(0.2),
            new TestRPMSpinLogic(drl, Constants.launcherRPMHighGoal)),


        new PrintCommand("at speed"),

        //Rumble when we are up to speed
        new InstantCommand(() -> {
          this.operator.setRumble(RumbleType.kLeftRumble, 1);
        }),

        
        new ParallelRaceGroup(
            new AutoConveyor(this.conveyor, -.1, -.3),
            new WaitUntilCommand(() -> this.weGoodToGo()),
            new WaitCommand(2)),


        new ParallelRaceGroup(
            new AutoConveyor(this.conveyor, 0.7, 0.9),
            new WaitUntilCommand(() -> !(this.weGoodToGo()))),

        
        //Stop Rumble when we are not up to speed
        new InstantCommand(() -> {
          this.operator.setRumble(RumbleType.kLeftRumble, 0);
        }),



        new InstantCommand(
            () -> {
              this.timer.stop();
              DriverStation.reportWarning("Time elapsed during spinup: " + this.timer.get(), false);
              this.timer.start();
            }),


        new InstantCommand(
            () -> {
              this.drl.rollingRPMAvg.emptyValues();
            },
            conveyor),


        new ParallelRaceGroup(
            new AutoConveyor(this.conveyor, -.1, -.3),
            new WaitUntilCommand(() -> this.weGoodToGo()),
            new WaitCommand(2)),


        new InstantCommand(() -> this.timer.stop()),


        new PrintCommand(" shot 2 timing: " + this.timer.get()),

        new ParallelCommandGroup(
            new AutoConveyor(conveyor, 0.7, 0.9), new AutoIntake(intake, Constants.intakePercent)));
  }

  @Override
  public void end(boolean interrupt) {
    this.drl.setZero();
    this.conveyor.runConveyor(0, 0);
    this.intake.runIntake(0);
    this.operator.setRumble(RumbleType.kLeftRumble, 0);
  }
}