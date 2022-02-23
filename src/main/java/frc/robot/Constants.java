// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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

  // shooter ports

  // the front of the robot is the RIO-side.
  // the "front" roller is the colson set, the
  // "back" set is the squishy wheels.

  // TODO CHANGE THESE IDs TO MATCH SPREADSHEET
  // BOTH IN CODE AND IN PHOENIX TUNER!

  public static final int ShooterWheelsMotor = 10;
  public static final int ShooterRollersMotor = 13;

  // conveyor motors
  public static final int ConveyorMotor = 9;
  public static final int FeederWheelMoter = 7;
  // TODO: Refactor name.

  // intake motor
  // this is going to be on a different CAN bus when the
  // CANivore shows up, so we need to specify that too.
  public static final int intakeMotor = 11;
  public static final String canivoreBus = "canivore";

  // climber motors
  public static final int leftClimberMotor = 8;
  public static final int rightClimberMotor = 12;

  // motion magic stuff for elevator (?)
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
