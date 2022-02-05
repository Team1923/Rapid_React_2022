// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DualRollerLauncherCommand.StopLauncher;

public class DualRollerLauncher extends SubsystemBase {

  public TalonFX frontMotor = new TalonFX(Constants.frontRollerMotor);
  public TalonFX backMotor = new TalonFX(Constants.backRollerMotor);

  public int front_kp, front_ki, front_kd, back_kp, back_ki, back_kd;
  public double front_setpt, back_setpt;

  /** Creates a new DualRollerLauncher. */
  public DualRollerLauncher() {

    // this is for in the event the robot reboots we need to explicitly set configurations
    // to avoid a latent state breaking stuff.
    frontMotor.configFactoryDefault();
    backMotor.configFactoryDefault();

    // this tells the falcon 500 to use their integrated encoders for velocity.
    // we can also set this to be other things, like a CANcoder, for example.
    frontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    backMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    frontMotor.setInverted(InvertType.InvertMotorOutput);
    backMotor.setInverted(InvertType.InvertMotorOutput);

    setDefaultCommand(new StopLauncher(this));
  }

  public void setFront(double spd) {
    DriverStation.reportWarning("Setting front roller to" + spd, false);
    frontMotor.set(ControlMode.PercentOutput, spd);
  }

  public void setBack(double spd) {
    DriverStation.reportWarning("Setting back roller to" + spd, false);
    backMotor.set(ControlMode.PercentOutput, spd);
  }
}
