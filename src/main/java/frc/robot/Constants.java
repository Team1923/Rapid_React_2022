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

  // Motor group reflection breaks with 2022 FRC/CTRE stuff, so in leiu of that it had to be
  // swapped.

  // DriveTrain Constants
  public static final int l1 = 1;
  public static final int l2 = 2;
  public static final int l3 = 3;
  public static final int r1 = 4;
  public static final int r2 = 5;
  public static final int r3 = 6;

  // Controller Ports
  public static final int driverPort = 0;
  public static final int operatorPort = 1;

  // dual roller launcher ports

  public static final int frontRollerMotor = 16;
  public static final int backRollerMotor = 17;

  // dual roller launcher constants

  // TODO CHANGE climber motor IDs
  public static final int climberLeft = 20;
  public static final int climberRight = 21;
}
