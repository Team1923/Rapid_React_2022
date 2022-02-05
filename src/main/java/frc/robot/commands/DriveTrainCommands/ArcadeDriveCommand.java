package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  private DriveTrainSubsystem drive;
  private XboxController driver_int;

  public ArcadeDriveCommand(DriveTrainSubsystem drive, XboxController driver) {
    this.driver_int = driver;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double zRot = driver_int.getRightX();
    double xSpeed = driver_int.getLeftY();

    // DriverStation.reportWarning("" + xSpeed + "\t" + zRot, false);

    this.drive.arcadeDrive(xSpeed, zRot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
