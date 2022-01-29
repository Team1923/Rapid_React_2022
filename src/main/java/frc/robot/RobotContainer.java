// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DualRollerLauncherCommand.DualRollerLauncherCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;

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

  public RobotContainer() {
    new JoystickButton(driver, XboxController.Button.kA.value)
        .whileHeld(new DualRollerLauncherCommand(drl));

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
