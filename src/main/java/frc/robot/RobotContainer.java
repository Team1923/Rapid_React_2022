// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autons.AlternativeTwoBallHighAuto;
import frc.robot.commands.Autons.DriveForwardAuto;
// import frc.robot.commands.Autons.FourBallAuto;
import frc.robot.commands.Autons.MirroredLow2BallAuto;
// import frc.robot.commands.Autons.MirroredThreeBallAuto;
import frc.robot.commands.Autons.MirroredTwoBallHighAuto;
import frc.robot.commands.Autons.OneBallHighAuto;
import frc.robot.commands.Autons.OneBallLowAuto;
import frc.robot.commands.Autons.PathweaverAutons.StraightLinePath;
import frc.robot.commands.Autons.PathweaverAutons.TwoBallAutoPW;
import frc.robot.commands.Autons.Test;
import frc.robot.commands.Autons.ThreeBallAuto;
import frc.robot.commands.Autons.TwoBallHighAuto;
import frc.robot.commands.Autons.TwoBallLowAuto;
import frc.robot.commands.DriveTrainCommands.ArcadeDriveCommand;
import frc.robot.commands.DriveTrainCommands.AutoDrive;
// import frc.robot.commands.DualRollerLauncherCommand.Exp.BumpFeeder;
import frc.robot.commands.DualRollerLauncherCommand.TeleopLauncherLowGoal;
import frc.robot.commands.ElevatorCommands.ElevatorCommand;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.commands.IntakeCommands.RunIntakeCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utilities.SpectrumAxisButton;
import java.io.IOException;
import java.nio.file.Path;

public class RobotContainer {

  // controllers
  public static XboxController driver = new XboxController(0);
  public static PS4Controller operator = new PS4Controller(1);

  public static DriveTrainSubsystem drive = new DriveTrainSubsystem();

  public static final ConveyorSubsystem conveyor = new ConveyorSubsystem();
  public static IntakeSubsystem intake = new IntakeSubsystem();
  public static ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static DualRollerLauncher drlSubsystem = new DualRollerLauncher();
  public static PigeonIMU pigeon = new PigeonIMU(Constants.pigeon);
  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  public NetworkTableEntry YAWangle = tuningTab.add("YAWAngle", 0).getEntry();

  // coach dashboard to choose an auto / show the auto.
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
  ShuffleboardLayout auto =
      coachTab.getLayout("Auto Setup", "List Layout").withPosition(0, 0).withSize(1, 5);

  public static boolean enableElevator = false;

  public static TwoBallHighAuto twoBallHighAuto =
      new TwoBallHighAuto(intake, drlSubsystem, drive, conveyor);
  public static OneBallLowAuto oneBallLowAuto =
      new OneBallLowAuto(intake, drive, conveyor, drlSubsystem);
  public static OneBallHighAuto oneBallHighAuto =
      new OneBallHighAuto(intake, drive, conveyor, drlSubsystem);
  public static DriveForwardAuto driveForwardAuto = new DriveForwardAuto(intake, drive, conveyor);

  //   public static MirroredThreeBallAuto threeballMirror =
  //       new MirroredThreeBallAuto(intake, drlSubsystem, drive, conveyor);

  //   public static FourBallAuto fourballAuto = new FourBallAuto(intake, drlSubsystem, drive,
  // conveyor);

  public static AlternativeTwoBallHighAuto alternativeTwoBallHighAuto =
      new AlternativeTwoBallHighAuto(intake, drlSubsystem, drive, conveyor);

  public static MirroredTwoBallHighAuto mirroredTwoBallHighAuto =
      new MirroredTwoBallHighAuto(intake, drlSubsystem, drive, conveyor);

  public static TwoBallLowAuto twoBallLowAuto =
      new TwoBallLowAuto(intake, drlSubsystem, drive, conveyor);

  public static MirroredLow2BallAuto mirroredLow2BallAuto =
      new MirroredLow2BallAuto(intake, drlSubsystem, drive, conveyor);

  public static ThreeBallAuto threeBallAuto =
      new ThreeBallAuto(intake, drlSubsystem, drive, conveyor);

  public static Test test = new Test(intake, drive, conveyor);

  public SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    // LiveWindow.disableAllTelemetry();

    // intake in (CROSS)
    new JoystickButton(operator, PS4Controller.Button.kCross.value)
        .whileHeld(new RunIntakeCommand(intake, operator, conveyor));

    // INTAKE OUT (SQUARE)
    // check about flipped values
    new JoystickButton(operator, PS4Controller.Button.kSquare.value)
        .whileHeld(new RunIntakeCommand(intake, operator, conveyor));

    // intake, feeder, conveyor wheels IN (depCIRCLE)
    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new RunIntakeCommand(intake, operator, conveyor));

    /*new JoystickButton(operator, PS4Controller.Button.kCircle.value)
    .whileHeld(new ConveyorCommand(conveyor, drlSubsystem));*/

    // shoot ball High Goal (TRIANGLE)

    // new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
    //     .toggleWhenPressed(new BumpFeeder(drlSubsystem, conveyor, operator));

    /* new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
    .toggleWhenPressed(new PerpetualCommand(new LaunchOneBallHigh(drlSubsystem, conveyor)));*/

    // shoot ball Low Goal (OPTION = 8)
    new JoystickButton(operator, 8).toggleWhenPressed(new TeleopLauncherLowGoal(drlSubsystem));

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
  }

  // auton Path from PathWeaver
  String trajectoryJSON = "paths/GoBack.wpilib.json";
  Trajectory trajectory = new Trajectory();

  public Command getAutonomousCommand() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("not opening", ex.getStackTrace());
    }

   // drive.reversed = true;
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            drive.getFeedForward(),
            Constants.kDriveKinematics,
            drive::getWheelSpeeds,
            drive.getLeftPidController(),
            drive.getRightPidController(),
            drive::tankDriveVolts,
            drive);

    drive.setPose(trajectory.getInitialPose());
   

    // return new StraightLinePath(intake, drlSubsystem, drive, conveyor);

    return new ParallelCommandGroup(
        new AutoIntake(intake, Constants.intakePercent),
        ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0))
    );

    //return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));

  }
}
