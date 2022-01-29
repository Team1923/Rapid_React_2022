package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;

public class DualRollerLauncherCommand extends CommandBase {

  private final DualRollerLauncher drl;

  ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

  NetworkTableEntry rpm;
  NetworkTableEntry currentVelocity;
  double TargetVelocity;

  // creating a drl command
  public DualRollerLauncherCommand(DualRollerLauncher drl) {
    rpm = tuneDualRollerTab.add("Velocity", 0).getEntry();
    currentVelocity = tuneDualRollerTab.add("Current Vel", 0).getEntry();

    this.drl = drl;
    TargetVelocity = rpm.getDouble(0);
  }

  // setting the front motors to the target RPM.
  public void execute() {
    // currentVelocity.setDouble(drl.frontMotor.getSelectedSensorVelocity(0));
    currentVelocity.setDouble(TargetVelocity);

    drl.setFrontVelocity(TargetVelocity);
    drl.setBackVelocity(TargetVelocity);
  }
}
