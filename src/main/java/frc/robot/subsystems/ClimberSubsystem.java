// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  private TalonFX leftMotor = new TalonFX(Constants.leftClimberMotor);

  private TalonFX rightMotor = new TalonFX(Constants.rightClimberMotor);

  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

  public ClimberSubsystem() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    // follow
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.InvertMotorOutput);

    leftMotor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    rightMotor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

    // configing motion magic
    leftMotor.selectProfileSlot(0, 0);
    leftMotor.config_kF(Constants.kIdx, Constants.kF, Constants.kTimeoutMs);
    leftMotor.config_kP(Constants.kIdx, Constants.kP, Constants.kTimeoutMs);
    leftMotor.config_kI(Constants.kIdx, Constants.kI, Constants.kTimeoutMs);
    leftMotor.config_kD(Constants.kIdx, Constants.kD, Constants.kTimeoutMs);
  }

  public void runClimber(double speed) {
    double target = 0;
    leftMotor.set(ControlMode.MotionMagic, target); // target is based on controller values
  }

  public TalonFX getLeftMotor() {
    return leftMotor;
  }

  public boolean isInRange(double currentValue, double target, double variation) {

    return (currentValue < (target + variation) && currentValue > (target - variation));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
