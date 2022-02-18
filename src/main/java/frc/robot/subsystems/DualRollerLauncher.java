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

  public WPI_TalonFX wheels = new WPI_TalonFX(Constants.wheels);
  public WPI_TalonFX rollers = new WPI_TalonFX(Constants.rollers);

  ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

  public NetworkTableEntry wheelsRPM = tuneDualRollerTab.add("Wheels RPM", 0).getEntry();
  public NetworkTableEntry rollersRPM = tuneDualRollerTab.add("Rollers RPM", 0).getEntry();

  public NetworkTableEntry frontnt;
  public NetworkTableEntry backnt;

  /** Creates a new DualRollerLauncher. */
  public DualRollerLauncher() {

    // this is for in the event the robot reboots we need to explicitly set configurations
    // to avoid a latent state breaking stuff.

    wheels.configFactoryDefault();
    rollers.configFactoryDefault();

    wheels.setInverted(InvertType.InvertMotorOutput);
    rollers.setInverted(InvertType.InvertMotorOutput);

    this.rollers.setNeutralMode(NeutralMode.Coast);
    this.wheels.setNeutralMode(NeutralMode.Coast);

    // #region Testing new PID CODE from 2/15/22

    wheels.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    rollers.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    wheels.configNominalOutputForward(0, 30);
    wheels.configNominalOutputReverse(0, 30);
    wheels.configPeakOutputForward(1, 30);
    wheels.configPeakOutputReverse(-1, 30);

    rollers.configNominalOutputForward(0, 30);
    rollers.configNominalOutputReverse(0, 30);
    rollers.configPeakOutputForward(1, 30);
    rollers.configPeakOutputReverse(-1, 30);

    // #endregion

    // this tells the falcon 500 to use their integrated encoders for velocity.
    // we can also set this to be other things, like a CANcoder, for example.

    // wheels.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    // rollers.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // configure P
    this.wheels.config_kP(0, 0.25, 30);
    this.rollers.config_kP(0, 0.18, 30);

    // configure I
    this.wheels.config_kI(0, 0, 30);
    this.rollers.config_kI(0, 0, 30);

    // configure D
    this.wheels.config_kD(0, 0, 30);
    this.rollers.config_kD(0, 0, 30);

    this.wheels.config_kF(0, .05, 30);
    this.rollers.config_kF(0, .057, 30);

    // setDefaultCommand(new DualRollerLauncherCommand(this, 0, 0)); // should stop it?
  }

  public void setWheels() {
    // DriverStation.reportWarning("Setting front roller to" + spd, false);
    double vel = wheelsRPM.getDouble(0) * 2048.0 / 600;

    // wheels.set(ControlMode.PercentOutput, 0.3);

    wheels.set(TalonFXControlMode.Velocity, vel);
  }

  public void setRollers() {
    // DriverStation.reportWarning("Setting back roller to" + spd, false);
    double vel = rollersRPM.getDouble(0) * 2048.0 / 600;
    // rollers.set(ControlMode.PercentOutput, 0.45);

    rollers.set(TalonFXControlMode.Velocity, vel);
  }

  public void setZero() {
    wheels.set(TalonFXControlMode.PercentOutput, 0);
    rollers.set(TalonFXControlMode.PercentOutput, 0);
  }
}
