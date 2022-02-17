// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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

  public NetworkTableEntry front_setpt =
      tuneDualRollerTab.add("Launcher front set point", 0).getEntry();
  public NetworkTableEntry front_kp = tuneDualRollerTab.add("Launcher front P value", 0).getEntry();
  public NetworkTableEntry front_ki = tuneDualRollerTab.add("Launcher front I value", 0).getEntry();
  public NetworkTableEntry front_kd = tuneDualRollerTab.add("Launcher front D value", 0).getEntry();
  double front_integral, front_error, front_derivative, front_previous_error, front_velocity = 0;
  public NetworkTableEntry frontRPM = tuneDualRollerTab.add("Front RPM", 0).getEntry();

  public NetworkTableEntry back_setpt =
      tuneDualRollerTab.add("Launcher back set point", 0).getEntry();
  public NetworkTableEntry back_kp = tuneDualRollerTab.add("Launcher back P value", 0).getEntry();
  public NetworkTableEntry back_ki = tuneDualRollerTab.add("Launcher back I value", 0).getEntry();
  public NetworkTableEntry back_kd = tuneDualRollerTab.add("Launcher back D value", 0).getEntry();
  double back_integral, back_error, back_derivative, back_previous_error, back_velocity = 0;
  public NetworkTableEntry backRPM = tuneDualRollerTab.add("Back RPM", 0).getEntry();

  public NetworkTableEntry frontnt;
  public NetworkTableEntry backnt;

  /** Creates a new DualRollerLauncher. */
  public DualRollerLauncher() {

    // this is for in the event the robot reboots we need to explicitly set configurations
    // to avoid a latent state breaking stuff.

    frontMotor.configFactoryDefault();
    backMotor.configFactoryDefault();

    frontMotor.setInverted(InvertType.InvertMotorOutput);
    backMotor.setInverted(InvertType.InvertMotorOutput);

    this.backMotor.setNeutralMode(NeutralMode.Coast);
    this.frontMotor.setNeutralMode(NeutralMode.Coast);

    // #region Testing new PID CODE from 2/15/22

    frontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    backMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    frontMotor.configNominalOutputForward(0, 30);
    frontMotor.configNominalOutputReverse(0, 30);
    frontMotor.configPeakOutputForward(1, 30);
    frontMotor.configPeakOutputReverse(-1, 30);

    backMotor.configNominalOutputForward(0, 30);
    backMotor.configNominalOutputReverse(0, 30);
    backMotor.configPeakOutputForward(1, 30);
    backMotor.configPeakOutputReverse(-1, 30);

    // #endregion

    // this tells the falcon 500 to use their integrated encoders for velocity.
    // we can also set this to be other things, like a CANcoder, for example.

    // frontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // backMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // configure P
    this.frontMotor.config_kP(0, 0.25, 30);
    this.backMotor.config_kP(0, 0.18, 30);

    // configure I
    this.frontMotor.config_kI(0, 0, 30);
    this.backMotor.config_kI(0, 0, 30);

    // configure D
    this.frontMotor.config_kD(0, 0, 30);
    this.backMotor.config_kD(0, 0, 30);

    this.frontMotor.config_kF(0, .05, 30);
    this.backMotor.config_kF(0, .057, 30);

    // setDefaultCommand(new DualRollerLauncherCommand(this, 0, 0)); // should stop it?

    frontnt = tuneDualRollerTab.add("front percent", 0).getEntry();
    backnt = tuneDualRollerTab.add("back percentout", 0).getEntry();
  }

  public void setFront() {
    // DriverStation.reportWarning("Setting front roller to" + spd, false);
    double vel = frontRPM.getDouble(0) * 2048.0 / 600;

    // frontMotor.set(ControlMode.PercentOutput, 0.3);

    frontMotor.set(TalonFXControlMode.Velocity, vel);
  }

  public void setBack() {
    // DriverStation.reportWarning("Setting back roller to" + spd, false);
    double vel = backRPM.getDouble(0) * 2048.0 / 600;
    // backMotor.set(ControlMode.PercentOutput, 0.45);

    backMotor.set(TalonFXControlMode.Velocity, vel);
  }

  public void setZero() {
    frontMotor.set(TalonFXControlMode.PercentOutput, 0);
    backMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
}
