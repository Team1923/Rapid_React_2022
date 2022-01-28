// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DualRollerLauncher extends SubsystemBase {

  public TalonFX frontMotor = new TalonFX(Constants.frontRollerMotor);
  public TalonFX backMotor = new TalonFX(Constants.backRollerMotor);

  public int front_kp, front_ki, front_kd, back_kp, back_ki, back_kd;
  public double front_setpt, back_setpt;

  ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

  NetworkTableEntry f_kp = tuneDualRollerTab.add("front kP", front_kp).getEntry();
  NetworkTableEntry f_ki = tuneDualRollerTab.add("front kI", front_ki).getEntry();
  NetworkTableEntry f_kd = tuneDualRollerTab.add("front kD", front_kd).getEntry();

  NetworkTableEntry b_kp = tuneDualRollerTab.add("back kP", back_kp).getEntry();
  NetworkTableEntry b_ki = tuneDualRollerTab.add("back kI", back_ki).getEntry();
  NetworkTableEntry b_kd = tuneDualRollerTab.add("back kD", back_kd).getEntry();

  NetworkTableEntry f_sp = tuneDualRollerTab.add("front setpoint", front_setpt).getEntry();
  NetworkTableEntry b_sp = tuneDualRollerTab.add("back setpoint", back_setpt).getEntry();

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
  }

  // this function is intended to be wrapped in a command.
  // this is not really ideal, as it will try to set the P,I,D,F and IZone terms _every call_.
  // will work for binding to a button, in theory.
  public void setPIDFront(double kP, double kI, double kD) {
    frontMotor.config_kP(0, kP);
    frontMotor.config_kI(0, kI);
    frontMotor.config_kD(0, kD);
    frontMotor.config_kF(0, 0); // we explicitly set this to avoid any shenanigans with feedforwards
    frontMotor.config_IntegralZone(0, 0.0); // not sure if this should be zero or not.
  }

  public void setFrontVelocity(double setpt) {

    frontMotor.set(ControlMode.Velocity, setpt);
  }

  public void setPIDBack(double kP, double kI, double kD) {
    backMotor.config_kP(0, kP);
    backMotor.config_kI(0, kI);
    backMotor.config_kD(0, kD);
    backMotor.config_kF(0, 0); // we explicitly set this to avoid any shenanigans with feedforwards
    backMotor.config_IntegralZone(0, 0.0); // not sure if this should be zero or not.
  }

  public void setBackVelocity(double setpt) {

    backMotor.set(ControlMode.Velocity, setpt);
  }

  @Override
  public void periodic() {

    // todo set getters to update internal values before setting it.
    // this will allow it to be actually usable.

    // front roller PID
    if (front_kp != f_kp.getDouble(0)) {
      front_kp = (int) f_kp.getDouble(0);
      b_kp.setDouble(back_kp);
    }

    if (front_ki != f_ki.getDouble(0)) {
      front_ki = (int) f_ki.getDouble(0);
      f_ki.setDouble(front_ki);
    }

    if (front_kd != f_kd.getDouble(0)) {
      front_kd = (int) f_kd.getDouble(0);
      f_kd.setDouble(front_kd);
    }

    // back roller PID
    if (back_kp != b_kp.getDouble(0)) {
      back_kp = (int) b_kp.getDouble(0);
      b_kp.setDouble(back_kp);
    }

    if (back_ki != b_ki.getDouble(0)) {
      back_ki = (int) b_ki.getDouble(0);
      b_ki.setDouble(back_ki);
    }

    if (back_kd != b_kd.getDouble(0)) { // gets, stores, then sets.
      back_kd = (int) b_kd.getDouble(0);
      b_kd.setDouble(back_kd);
    }

    // front and back setpoint

    if (front_setpt != f_sp.getDouble(0)) {
      front_setpt = f_sp.getDouble(0);
    }

    if (back_setpt != b_sp.getDouble(0)) {
      back_setpt = b_sp.getDouble(0);
    }
  }
}
