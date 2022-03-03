// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autons.AlternativeTwoBallHighAuto;
import frc.robot.commands.Autons.DriveForwardAuto;
import frc.robot.commands.Autons.OneBallHighAuto;
import frc.robot.commands.Autons.OneBallLowAuto;
import frc.robot.commands.Autons.TwoBallHighAuto;
import frc.robot.commands.ConveyorCommands.ConveyorCommand;
import frc.robot.commands.DriveTrainCommands.ArcadeDriveCommand;
import frc.robot.commands.DualRollerLauncherCommand.TeleopRunDRLHigh;
import frc.robot.commands.DualRollerLauncherCommand.TeleopRunDRLLow;
import frc.robot.commands.ElevatorCommands.ElevatorCommand;
import frc.robot.commands.IntakeCommands.RunIntakeCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.SpectrumAxisButton;

public class RobotContainer {

  // controllers
  public static XboxController driver = new XboxController(0);
  public static PS4Controller operator = new PS4Controller(1);

  public static DriveTrainSubsystem drive = new DriveTrainSubsystem();
  public static final DualRollerLauncher drl = new DualRollerLauncher();

  public static final ConveyorSubsystem conveyor = new ConveyorSubsystem();
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static PigeonIMU pigeon = new PigeonIMU(Constants.pigeon);
  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  public NetworkTableEntry YAWangle = tuningTab.add("YAWAngle", 0).getEntry();

  public static boolean enableElevator = false;

  public static TwoBallHighAuto twoBallHighAuto = new TwoBallHighAuto(intake, drl, drive, conveyor);
  public static OneBallLowAuto oneBallLowAuto = new OneBallLowAuto(intake, drive, conveyor, drl);
  public static OneBallHighAuto oneBallHighAuto = new OneBallHighAuto(intake, drive, conveyor, drl);
  public static DriveForwardAuto driveForwardAuto =
      new DriveForwardAuto(intake, drive, conveyor, drl);

  public static AlternativeTwoBallHighAuto alternativeTwoBallHighAuto =
      new AlternativeTwoBallHighAuto(intake, drl, drive, conveyor);

  public SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {

    LiveWindow.disableAllTelemetry();

    // intake in (CROSS)
    new JoystickButton(operator, PS4Controller.Button.kCross.value)
        .whileHeld(new RunIntakeCommand(intake, operator));

    // INTAKE OUT (SQUARE)
    // check about flipped values
    new JoystickButton(operator, PS4Controller.Button.kSquare.value)
        .whileHeld(new RunIntakeCommand(intake, operator));

    // intake, feeder, conveyor wheels IN (CIRCLE)
    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new RunIntakeCommand(intake, operator));
    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new ConveyorCommand(conveyor, drl));

    // shoot ball High Goal (TRIANGLE)
    new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
        .toggleWhenPressed(new TeleopRunDRLHigh(drl));

    // shoot ball Low Goal (OPTION = 8)
    new JoystickButton(operator, 8).toggleWhenPressed(new TeleopRunDRLLow(drl));

    // drive (arcade)
    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kLeftY.value,
            0.05,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new ArcadeDriveCommand(drive, driver));

    // CLIMBER

    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kLeftTrigger.value,
            0.1,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new ElevatorCommand(elevator, driver));

    new SpectrumAxisButton(
            driver,
            XboxController.Axis.kRightTrigger.value,
            0.1,
            SpectrumAxisButton.ThresholdType.DEADBAND)
        .whileActiveOnce(new ElevatorCommand(elevator, driver));

    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    YAWangle.setDouble(ypr[0]);

    chooser.setDefaultOption("OneBallLowAuto", oneBallLowAuto);
    chooser.addOption("TwoBallHighAuto", twoBallHighAuto);
    chooser.addOption("Drive Forward Auto", driveForwardAuto);
    chooser.addOption("OneBallHighAuto", oneBallHighAuto);
    chooser.addOption("Death Trap 2 ball", alternativeTwoBallHighAuto);
    SmartDashboard.putData(chooser);
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
    // return null;
  }
}
