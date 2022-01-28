package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  public ArcadeDriveCommand(DriveTrainSubsystem drive) {
    addRequirements(drive);
  }

  /*this command can return true at any time, because it'll be a continuous, default command.
  We don't need to worry about the isFinished function for a "stateless" command, that commands a subsystem without
  much smarts, but if we were say, chaining a launcher command, we *would* need to write some logic
  to determine whether the command can be like "yep we did the thing we need to."

  See: https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html#simple-command-example
  */

  // second command to do zeroing and stuff.
  public ArcadeDriveCommand(DriveTrainSubsystem drive, double xSpeed, double zRot) {
    addRequirements(drive);
    drive.arcadeDrive(xSpeed, zRot);
  }

  @Override
  public boolean isFinished() {

    return true;
  }
}
