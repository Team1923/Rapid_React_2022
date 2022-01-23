package frc.robot.commands.DriveTrainCommands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;

public class DualRollerLauncherCommand extends CommandBase{

    private final DualRollerLauncher drl;

    ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

    

    NetworkTableEntry rpm = tuneDualRollerTab.add("Velocity", 0).getEntry();
    



    //creating a drl command
    public DualRollerLauncherCommand(DualRollerLauncher drl){
        
        this.drl = drl;
        
    }


    //setting the front motors to the target RPM.
    public void execute(){
        
        double TargetVelocity = rpm.getDouble(0);

        drl.setFrontVelocity(TargetVelocity);
        
    }



    
    
}
