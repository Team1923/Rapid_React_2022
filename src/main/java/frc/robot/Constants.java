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

  // motion magic stuff
  public static final int kIdx = 0;
  public static final int kPIDloopIdx = 0;
  public static final int kTimeoutMs = 0;
  public static final double kP = 0.2;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kF = 0.2;
  public static final double kIzone = 0;
  public static final double kPeakOutput = 1.0;

  // DriveTrain Constants
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

  public static final int frontRollerMotor = 7;
  public static final int backRollerMotor = 8;

  // conveyor motors

  public static final int beltRollerMotor = 9;
  public static final int wheelConveyorMotor = 10;
  // TODO: Refactor name.

  // intake motor
  public static final int intakeMotor = 11;

  // climber motors

  public static final int leftClimberMotor = 12;
  public static final int rightClimberMotor = 13;
}
