package frc.robot.commands.DriveTrainCommands;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utilities.controller.Axis;

public class TankDriveCommand extends DriveTrainSubsystem {

  private Double LeftStickValue, RightStickValue;

  public TankDriveCommand(Axis LeftStick, Axis RightStick) {

    LeftStickValue = LeftStick.get();
    RightStickValue = RightStick.get();
  }

  public void excute() {
    setSpeed(RightStickValue, LeftStickValue);
  }
}
