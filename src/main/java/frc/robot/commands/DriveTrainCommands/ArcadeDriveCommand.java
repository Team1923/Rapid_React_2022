package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  private DriveTrainSubsystem drive;

  public ArcadeDriveCommand(DriveTrainSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    drive.arcadeDrive(0, 0);
    /* by default we stop the drivetrain.  this _will_ have issues
    until we switch to closed loop control and add ramp rates. */
  }

  public ArcadeDriveCommand(DriveTrainSubsystem drive, double xSpeed, double zRot) {
    this.drive = drive;
    addRequirements(drive);
    drive.arcadeDrive(xSpeed, zRot);
  }

  /* The isFinished function will return false forever, because we're using this as a default command and cannot, by definition, ever be done.
  It simply takes a lower priority by virtue of that, and cancels the default command if something *else*
  gets scheduled / needs the subsystem 'drive', which our drive command will.

  See: https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html#simple-command-example
  */

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean i) {
    this.drive.arcadeDrive(0,0);

  }
}
