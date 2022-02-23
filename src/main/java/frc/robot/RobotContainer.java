// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autons.DriveForwardAuto;
import frc.robot.commands.Autons.OneBallLowAuto;
import frc.robot.commands.Autons.TwoBallHighAuto;
import frc.robot.commands.ConveyorCommands.ConveyorCommand;
import frc.robot.commands.DriveTrainCommands.ArcadeDriveCommand;
import frc.robot.commands.DualRollerLauncherCommand.RunDRLCommand;
import frc.robot.commands.IntakeCommands.RunIntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;
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
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static ClimberSubsystem climber = new ClimberSubsystem();

  public static boolean enableClimber = false;

  public static TwoBallHighAuto twoBallHighAuto = new TwoBallHighAuto(intake, drl, drive, conveyor);
  public static OneBallLowAuto oneBallLowAuto = new OneBallLowAuto(intake, drive, conveyor, drl);
  public static DriveForwardAuto driveForwardAuto =
      new DriveForwardAuto(intake, drive, conveyor, drl);

  public SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {

    // intake in (CIRCLE)
    new JoystickButton(operator, PS4Controller.Button.kCross.value)
        .whileHeld(new RunIntakeCommand(intake, operator));

    new JoystickButton(operator, PS4Controller.Button.kSquare.value)
        .whileHeld(new RunIntakeCommand(intake, operator));

    // intake, feeder, conveyor wheels IN (CIRCLE)
    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new RunIntakeCommand(intake, operator));
    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new ConveyorCommand(conveyor, drl));

    // shoot ball (TRIANGLE)
    new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
        .toggleWhenPressed(new RunDRLCommand(drl));

    // new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
    //      .toggleWhenPressed(new DRLTEST2(drl2));

    // drive (arcade)
    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kLeftY.value,
            0.05,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new ArcadeDriveCommand(drive, driver));

    // CLIMBER

    // new SpectrumAxisButton(
    //         driver,
    //         XboxController.Axis.kLeftTrigger.value,
    //         0.1,
    //         SpectrumAxisButton.ThresholdType.DEADBAND)
    //     .whileActiveOnce(new ClimberTest(climber, driver));

    // new SpectrumAxisButton(
    //         driver,
    //         XboxController.Axis.kRightTrigger.value,
    //         0.1,
    //         SpectrumAxisButton.ThresholdType.DEADBAND)
    //     .whileActiveOnce(new ClimberTest(climber, driver));

    chooser.setDefaultOption("OneBallLowAuto", oneBallLowAuto);
    chooser.addOption("TwoBallHighAuto", twoBallHighAuto);
    chooser.addOption("Drive Forward Auto", driveForwardAuto);
    SmartDashboard.putData(chooser);
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
    // return null;
  }
}
