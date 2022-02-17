// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climber.ClimberTest;
import frc.robot.commands.Conveyor.ConveyorTest;
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

  public static boolean enableClimber = false;

  public RobotContainer() {

    // intake in (CIRCLE)
    new JoystickButton(operator, PS4Controller.Button.kCross.value)
        .whileHeld(new IntakeTest(intake, operator));

    new JoystickButton(operator, PS4Controller.Button.kSquare.value)
        .whileHeld(new IntakeTest(intake, operator));

    // intake, feeder, conveyor wheels IN (CIRCLE)
    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new IntakeTest(intake, operator));
    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new ConveyorTest(conveyor));

    // shoot ball (TRIANGLE)
    new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
        .toggleWhenPressed(new DRLTestRun(drl));

    // new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
    //      .toggleWhenPressed(new DRLTEST2(drl2));

    // drive (arcade)
    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kLeftY.value,
            0.05,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new DriveTest(drive, driver));

    // CLIMBER

    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kLeftTrigger.value,
            0.1,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new ClimberTest(climber, driver));

    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kRightTrigger.value,
            0.1,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new ClimberTest(climber, driver));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
