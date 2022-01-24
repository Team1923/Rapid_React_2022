package frc.robot.commands.DriveTrainCommands;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;

public class DualRollerLauncherCommand extends CommandBase{

    private final DualRollerLauncher drl;

    ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

    

    NetworkTableEntry rpm = tuneDualRollerTab.add("Velocity", 0).getEntry();
    NetworkTableEntry currentVelocity = tuneDualRollerTab.add("Velocity", 0).getEntry();
    double TargetVelocity;




    //creating a drl command
    public DualRollerLauncherCommand(DualRollerLauncher drl){
        this.drl = drl;
        TargetVelocity = rpm.getDouble(0);
        
    }

    public DualRollerLauncherCommand(DualRollerLauncher drl, int vel){
        this.drl = drl;
        TargetVelocity = vel;
        
    }


    //setting the front motors to the target RPM.
    public void execute(){
       currentVelocity.setDouble(drl.frontMotor.getSelectedSensorVelocity());
        

        drl.setFrontVelocity(TargetVelocity);
        
    }



    
    
}
