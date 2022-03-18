// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

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

  public static final SupplyCurrentLimitConfiguration drivetrainCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 60, 80, .2);

  // Launcher IDs
  public static final int WheelMotorCanIDA = 10;
  public static final int WheelMotorCANIDB = 7;

  // Conveyor IDs
  public static final int ConveyorMotor = 13;
  public static final int FeederWheelMotor = 9;

  // Intake ID
  public static final int intakeMotor = 11;
  public static final SupplyCurrentLimitConfiguration intakeCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 30, 40, .2);

  // Climber IDs
  public static final int leftClimberMotor = 8;
  public static final int rightClimberMotor = 12;

  // intake speeds.
  public static final double intakePercent = 0.9;

  // conveyor speeds.
  public static final double conveyorPerent = 0.3;
  public static final double feederWheelsPercent = 0.3;

  // to be changed, RPM targets for high and low shots at fender.
  public static final double launcherRPMLowGoal =
      2400; // TOOD: Verify.  This is mathematically similar to the "working" shot we had at 1600RPM
  // * 1.5 -> 2400.
  public static final int launcherRPMHighGoal = 4050;
  public static final int launcherRPMTolerance = 45; // experimentally found.

  // pigeon ID
  public static final int pigeon = 14;

  // climber elevator spool-out prevention
  // will have some slack(not intended to be precise)
  public static final double elevatorMaxRevs = 32;
  public static final SupplyCurrentLimitConfiguration elevatorCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);
}
