package frc.robot.commands.DriveTrainCommands;

import javax.sql.rowset.serial.SerialStruct;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.utilities.controller.Axis;

public class ArcadeDriveCommand extends DriveTrainSubsystem{

    private Axis LeftStick, RightStick;


    public ArcadeDriveCommand(Axis LeftStick, Axis RightStick){

        this.LeftStick = LeftStick;
        this.RightStick = RightStick;
    }



    public void excute(){
        setSpeed(LeftStick.add(RightStick.map(value -> value * Math.pow(1 - 0.5 * LeftStick.get(), 2))).clamp().get(), 
            LeftStick.subtract(RightStick.map(value -> value * Math.pow(1 - 0.5 * LeftStick.get(), 2))).clamp().get());
    }
}
    

