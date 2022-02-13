// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climber.ClimberTest;
import frc.robot.commands.Conveyor.ConveyorTest;
import frc.robot.commands.Conveyor.ConveyorTest2;
import frc.robot.commands.Conveyor.ConveyorTest3;
import frc.robot.commands.DriveTrainCommands.DriveTest;
import frc.robot.commands.DualRollerLauncherCommand.DRLTestRun;
import frc.robot.commands.Intake.IntakeTest;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.SpectrumAxisButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // controllers
  public static XboxController driver = new XboxController(0);
  public static PS4Controller operator = new PS4Controller(1);

  public static DriveTrainSubsystem drive = new DriveTrainSubsystem();
  public static final DualRollerLauncher drl = new DualRollerLauncher();
  public static final ConveyorSubsystem conveyor = new ConveyorSubsystem();
  public static Intake intake = new Intake();
  public static ClimberSubsystem climber = new ClimberSubsystem();

  public RobotContainer() {
    // new JoystickButton(driver, XboxController.Button.kA.value)
    //     /*.whileHeld(new DualRollerLauncherCommand(drl));*/
    //     .whileActiveContinuous(new DualRollerLauncherCommand(drl));

    // new ArcadeOldCommand(drive, driver.getLeftY(), driver.getRightX());
    // new ConveyorCommand(conveyor, operator.getLeftY(), operator.getRightY());

    // SmartDashboard.putData("Run Launcher", new DualRollerLauncherCommand(drl, .4, .5));

    // SmartDashboard.putData("Run Launcher", new DRLTestRun(drl));

    new JoystickButton(driver, XboxController.Button.kA.value).whileHeld(new
    IntakeTest(intake));
    new JoystickButton(driver, XboxController.Button.kA.value).whileHeld(new
    ConveyorTest(conveyor));

    // new JoystickButton(operator, PS4Controller.Button.kCross.value)
    //     .whileHeld(
    //         new ParallelCommandGroup(
    //             new IntakeTest(new Intake()), new DRLTestRun(new DualRollerLauncher())));

    new JoystickButton(driver, XboxController.Button.kB.value)
        .whileHeld(new ConveyorTest2(conveyor));

    new JoystickButton(driver, XboxController.Button.kY.value)
        .whileHeld(new DRLTestRun(drl));

    new JoystickButton(driver, XboxController.Button.kX.value)
        .whileHeld(new ConveyorTest3(conveyor));

    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kLeftY.value,
            0.05,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new DriveTest(drive, driver));

    new SpectrumAxisButton(
            operator,
            PS4Controller.Axis.kLeftY.value,
            0.05,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new ClimberTest(climber, operator));
    
    // new JoystickButton(driver, XboxController.Button.kA.value)
    //     .whileActiveOnce(new RunIntakeCommand(intake, 0.7));

    // new JoystickButton(driver, XboxController.Button.kStart.value)
    //     .whileActiveOnce(new DualRollerLauncherCommand(drl, .4, .5));

    // this stuff is mirrored mostly from this.
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
    /*
    new JoystickButton(driver, XboxController.Axis.kLeftY.value)
        .and(new JoystickButton(driver, XboxController.Axis.kRightX.value))
        .whenActive(new ArcadeDriveCommand(drive, driver.getLeftY(), driver.getRightX()));*/
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
