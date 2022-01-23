package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  private TalonFX r1 = new TalonFX(Constants.r1);
  private TalonFX r2 = new TalonFX(Constants.r2);
  private TalonFX r3 = new TalonFX(Constants.r3);
  private TalonFX l1 = new TalonFX(Constants.l1);
  private TalonFX l2 = new TalonFX(Constants.l2);
  private TalonFX l3 = new TalonFX(Constants.l3);

  public DriveTrainSubsystem() {
    r2.follow(r1);
    r3.follow(r1);

    // TODO confirm inversion behavior.
    // Taken from example here:
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/DifferentialDrive/src/main/java/frc/robot/Robot.java#L143
    l2.follow(l1);
    l2.setInverted(InvertType.FollowMaster);
    l3.follow(l1);
    l2.setInverted(InvertType.FollowMaster);
  }

  public void setSpeed(double rightSpeed, double leftSpeed) {
    r1.set(ControlMode.PercentOutput, rightSpeed);
    l1.set(ControlMode.PercentOutput, leftSpeed);
  }
}
