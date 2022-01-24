package frc.robot.commands.DualRollerLauncherCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DualRollerLauncher;

public class DualRollerLauncherCommandSet0 extends CommandBase{

    private final DualRollerLauncher drl;
    double TargetVelocity;




    //creating a drl command
    public DualRollerLauncherCommandSet0(DualRollerLauncher drl){

        this.drl = drl;
        TargetVelocity = 0;
        
    }

    //setting the front motors to the target RPM.
    public void execute(){
      // currentVelocity.setDouble(drl.frontMotor.getSelectedSensorVelocity(0));
     //currentVelocity.setDouble(TargetVelocity);
        

        drl.setFrontVelocity(TargetVelocity);
        drl.setBackVelocity(TargetVelocity);
        
        
    }



    
    
}
