// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DualRollerLauncher extends SubsystemBase {

  public WPI_TalonFX frontMotor = new WPI_TalonFX(Constants.frontRollerMotor);
  public WPI_TalonFX backMotor = new WPI_TalonFX(Constants.backRollerMotor);
  

  ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

  public NetworkTableEntry front_setpt = tuneDualRollerTab.add("Launcher front set point", 0).getEntry();
  public NetworkTableEntry front_kp = tuneDualRollerTab.add("Launcher front P value", 0).getEntry();
  public  NetworkTableEntry front_ki = tuneDualRollerTab.add("Launcher front I value", 0).getEntry();
  public NetworkTableEntry front_kd = tuneDualRollerTab.add("Launcher front D value", 0).getEntry();
  double front_integral, front_error, front_derivative, front_previous_error, front_velocity = 0;

  public NetworkTableEntry back_setpt = tuneDualRollerTab.add("Launcher back set point", 0).getEntry();
  public NetworkTableEntry back_kp = tuneDualRollerTab.add("Launcher back P value", 0).getEntry();
  public NetworkTableEntry back_ki = tuneDualRollerTab.add("Launcher back I value", 0).getEntry();
  public  NetworkTableEntry back_kd = tuneDualRollerTab.add("Launcher back D value", 0).getEntry();
  double back_integral, back_error, back_derivative, back_previous_error, back_velocity = 0;

  public NetworkTableEntry frontnt;
  public NetworkTableEntry backnt;

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

    this.backMotor.setNeutralMode(NeutralMode.Coast);
    this.frontMotor.setNeutralMode(NeutralMode.Coast);

    // setDefaultCommand(new DualRollerLauncherCommand(this, 0, 0)); // should stop it?

    frontnt = tuneDualRollerTab.add("front percent", 0).getEntry();
    backnt = tuneDualRollerTab.add("back percentout", 0).getEntry();
  }

  // public double frontPID() {
  //   front_error = front_setpt.getDouble(0) - frontMotor.getSelectedSensorVelocity();
  //   front_integral += (front_error * 0.02);
  //   front_derivative = (front_error - front_previous_error) / 0.02;
  //   front_velocity =
  //       front_kp.getDouble(0) * front_error
  //           + front_ki.getDouble(0) * front_integral
  //           + front_kd.getDouble(0) * front_derivative;

  //   return front_velocity;
  // }

  // public double backPID() {
  //   back_error = back_setpt.getDouble(0) - backMotor.getSelectedSensorVelocity();
  //   back_integral += (back_error * 0.02);
  //   back_derivative = (back_error - back_previous_error) / 0.02;
  //   back_velocity =
  //       back_kp.getDouble(0) * back_error
  //           + back_ki.getDouble(0) * back_integral
  //           + back_kd.getDouble(0) * back_derivative;
  //   return back_velocity;
  // }

  public void setFront(double spd) {
    // DriverStation.reportWarning("Setting front roller to" + spd, false);
    frontMotor.set(ControlMode.Velocity, spd);
  }

  public void setBack(double spd) {
    // DriverStation.reportWarning("Setting back roller to" + spd, false);
    backMotor.set(ControlMode.Velocity, spd);
  }
}
