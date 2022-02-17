// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.leftClimberMotor);

  public WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.rightClimberMotor);

  public SupplyCurrentLimitConfiguration supplyCurrentLimitConfigurationWhenClimbing =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

  public SupplyCurrentLimitConfiguration supplyCurrentLimitConfigurationWhenNotClimbing =
      new SupplyCurrentLimitConfiguration(true, 5, 10, 0.25);

  public ClimberSubsystem() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    // follow
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.InvertMotorOutput);

    leftMotor.configSupplyCurrentLimit(supplyCurrentLimitConfigurationWhenNotClimbing);
    rightMotor.configSupplyCurrentLimit(supplyCurrentLimitConfigurationWhenNotClimbing);

    // set up encoder logic.
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    // configing motion magic
    leftMotor.selectProfileSlot(0, 0);
    leftMotor.config_kF(Constants.kIdx, Constants.kF, Constants.kTimeoutMs);
    leftMotor.config_kP(Constants.kIdx, Constants.kP, Constants.kTimeoutMs);
    leftMotor.config_kI(Constants.kIdx, Constants.kI, Constants.kTimeoutMs);
    leftMotor.config_kD(Constants.kIdx, Constants.kD, Constants.kTimeoutMs);
  }

  public void runClimber(double leftSpeed, double rightSpeed) {
    // double target = driver.getRightTriggerAxis() * 2048 * 10;
    // leftMotor.set(ControlMode.MotionMagic, target);
    if (rightSpeed > 0.1) {
      leftMotor.set(ControlMode.PercentOutput, -.5 * rightSpeed);
    }
    if (leftSpeed > 0.1) {
      leftMotor.set(ControlMode.PercentOutput, .5 * leftSpeed);
    }
  }

  public void setZero() {
    this.leftMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isInRange(double currentValue, double target, double variation) {

    return (currentValue < (target + variation) && currentValue > (target - variation));
  }

  public double encVal() {
    return leftMotor.getSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
