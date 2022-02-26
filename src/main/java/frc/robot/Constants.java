// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  // Controller Ports
  public static final int driverPort = 0;
  public static final int operatorPort = 1;

  // DriveTrain CAN IDs
  public static final int l1 = 1;
  public static final int l2 = 2;
  public static final int l3 = 3;
  public static final int r1 = 4;
  public static final int r2 = 5;
  public static final int r3 = 6;

  // Shooter IDs
  public static final int ShooterWheelsMotor = 10;
  public static final int ShooterRollersMotor = 7;

  // Conveyor IDs
  public static final int ConveyorMotor = 13;
  public static final int FeederWheelMoter = 9;

  // Intake ID
  public static final int intakeMotor = 11;
  public static final String canivoreBus = "Default Name";

  // Climber IDs
  public static final int leftClimberMotor = 8;
  public static final int rightClimberMotor = 12;

  // Shuffleboard values
  public static final double intakePercent = -0.9;
  public static final double conveyorPerent = -0.9;
  public static final double feederWheelsPercent = -0.9;
  public static final double shooterWheelsRPM = 2700;
  public static final double shooterRollerRPM = 900;

  // motion magic stuff for elevator
  public static final int kIdx = 0;
  public static final int kPIDloopIdx = 0;
  public static final int kTimeoutMs = 0;
  public static final double kP = 0.2;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0.2;
  public static final double kIzone = 0;
  public static final double kPeakOutput = 1.0;
}
