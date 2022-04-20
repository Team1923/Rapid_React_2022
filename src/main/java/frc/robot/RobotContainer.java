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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autons.AutoChooser;
import frc.robot.commands.ConveyorCommands.ManualConveyor;
import frc.robot.commands.DriveTrainCommands.ArcadeDriveCommand;
import frc.robot.commands.DualRollerLauncherCommand.Exp.BumpFeederLowGoal;
import frc.robot.commands.DualRollerLauncherCommand.Exp.LaunchBalls;
import frc.robot.commands.ElevatorCommands.ElevatorCommand;
import frc.robot.commands.ElevatorCommands.FourBar;
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

  public DriveTrainSubsystem drive = new DriveTrainSubsystem();

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

  public SendableChooser<Command> chooser = new SendableChooser<>();

  AutoChooser selector;

  public Command initializeAuto(AutoChooser selector) {
    System.out.println("REACHED");
    this.selector = selector;
    return selector.startMode(intake, drlSubsystem, drive, conveyor);
  }

  public RobotContainer() {
    LiveWindow.disableAllTelemetry();

    // intake in (CROSS)
    new JoystickButton(operator, PS4Controller.Button.kCross.value)
        .whileHeld(new RunIntakeCommand(intake, operator, conveyor));

    // INTAKE OUT (SQUARE)
    // check about flipped values
    new JoystickButton(operator, PS4Controller.Button.kSquare.value)
        .whileHeld(new RunIntakeCommand(intake, operator, conveyor));

    // intake, feeder, conveyor wheels IN (CIRCLE)
    // new JoystickButton(operator, PS4Controller.Button.kCircle.value)
    //     .whileHeld(new RunCommand(() ->
    // {conveyor.runConveyorVel(UnitConversion.RPMtoNativeUnits(6000),UnitConversion.RPMtoNativeUnits(6000));}));

    new JoystickButton(operator, PS4Controller.Button.kCircle.value)
        .whileHeld(new ManualConveyor(conveyor, intake));

    // Servo Code
    new JoystickButton(driver, XboxController.Button.kStart.value)
        .whenPressed(new FourBar(elevator, driver));

    // shoot ball High Goal (TRIANGLE)

    // new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
    //     .toggleWhenPressed(new BumpFeederHighGoal(drlSubsystem, conveyor, operator));

    // experimental
    new JoystickButton(operator, PS4Controller.Button.kTriangle.value)
        .toggleWhenPressed(new LaunchBalls(drlSubsystem, conveyor, intake));

    // shoot ball Low Goal (OPTION = 8)
    new JoystickButton(operator, 8)
        .toggleWhenPressed(new BumpFeederHighGoal(drlSubsystem, conveyor, operator));

    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whenPressed(new FourBar(elevator, driver));

    // Shooter single motor movement

    // new SpectrumAxisButton(
    //     operator,
    //     PS4Controller.Axis.kLeftX.value, 0.05,
    //     SpectrumAxisButton.ThresholdType.DEADBAND).whileActiveOnce(new RunCommand(()->{
    //         drlSubsystem.runOneMotor(2000);
    //     }));

    //     new SpectrumAxisButton(
    //         operator,
    //         PS4Controller.Axis.kRightX.value, 0.05,
    //         SpectrumAxisButton.ThresholdType.DEADBAND).whileActiveOnce(new RunCommand(()->{
    //             drlSubsystem.runOtherMotor(2000);
    //         }));

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
}
