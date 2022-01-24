// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveTrainCommands.TankDriveCommand;
import frc.robot.commands.DualRollerLauncherCommand.DualRollerLauncherCommand;
import frc.robot.commands.DualRollerLauncherCommand.DualRollerLauncherCommandSet0;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.utilities.controller.PS4Controller;
import frc.robot.utilities.controller.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final DualRollerLauncher drl = new DualRollerLauncher();

  public final XboxController driver = new XboxController(Constants.driverPort);
  public final PS4Controller operator = new PS4Controller(Constants.operatorPort);

  public RobotContainer() {

    //starting the roller wheel
    driver.a.whileHeld(new DualRollerLauncherCommand(drl));
    //Stopping the roller wheel
    driver.b.whileHeld(new DualRollerLauncherCommandSet0(drl));

    // Starting the DriveTrain
    new TankDriveCommand(driver.rightStick.y, driver.leftStick.y);
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
