package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.utilities.command.MotorSubsystem;
import frc.robot.utilities.motor.Motor;


public class DriveTrainSubsystem extends MotorSubsystem {
    
    private Motor Left = (Motor)Constants.LeftMotors;
    private Motor Right = (Motor)Constants.RightMotors;




    public void setSpeed(double rightSpeed, double leftSpeed){

        Left.setSpeed(leftSpeed);
        Right.setSpeed(rightSpeed);


    }
    



}
