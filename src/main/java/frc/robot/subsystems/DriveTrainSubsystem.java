package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveTrainCommands.ArcadeDriveCommand;

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

    // this is set due to the behavior seen on MKI 2022 "P", may need adjustment.
    l1.setInverted(InvertType.InvertMotorOutput);

    setDefaultCommand(new ArcadeDriveCommand(this));
  }

  public void setSpeed(double rightSpeed, double leftSpeed) {
    r1.set(ControlMode.PercentOutput, rightSpeed);
    l1.set(ControlMode.PercentOutput, leftSpeed);
  }

  // arcade drive function to the subsystem, not the command.  called with an external command.
  // taken from 3847:
  // https://github.com/Spectrum3847/Ultraviolet-2020/blob/master/src/main/java/frc/robot/subsystems/Drivetrain.java#L128
  public void arcadeDrive(double xSpeed, double zRotation) {
    xSpeed = limit(xSpeed);

    // Make the deadzone bigger if we are driving fwd or backwards and not turning in place
    if (Math.abs(xSpeed) > 0.1 && Math.abs(zRotation) < 0.05) {
      zRotation = 0;
    }

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
    if (xSpeed == 0) {
      leftMotorOutput = zRotation;
      rightMotorOutput = -zRotation;
    } else {
      if (xSpeed >= 0.0) {
        // First quadrant, else second quadrant
        if (zRotation >= 0.0) {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
        } else {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
        }
      } else {
        // Third quadrant, else fourth quadrant
        if (zRotation >= 0.0) {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
        } else {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
        }
      }
    }

    r1.set(ControlMode.PercentOutput, limit(rightMotorOutput));
    l1.set(ControlMode.PercentOutput, limit(leftMotorOutput));
  }

  // taken from 3847 to make their arcade drive function work.
  // https://github.com/Spectrum3847/Ultraviolet-2020/blob/master/src/main/java/frc/robot/subsystems/Drivetrain.java#L118
  protected double limit(double value) {
    if (value > 1.0) {
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value;
  }
}
