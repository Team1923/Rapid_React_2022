package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;

public class DualRollerLauncherCommand extends CommandBase {

  private final DualRollerLauncher drl;

  ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

  NetworkTableEntry frontnt;
  NetworkTableEntry backnt;
  double TargetVelocity;

  // creating a drl command
  public DualRollerLauncherCommand(DualRollerLauncher drl) {
    addRequirements(drl);
    frontnt = tuneDualRollerTab.add("front percentout", 0).getEntry();
    backnt = tuneDualRollerTab.add("back percentout", 0).getEntry();

    this.drl = drl;
  }

  // setting the front motors to the target RPM.
  @Override
  public void execute() {
    // currentVelocity.setDouble(drl.frontMotor.getSelectedSensorVelocity(0));

    this.drl.setFront(frontnt.getDouble(0));
    this.drl.setBack(backnt.getDouble(0));
  }
}
