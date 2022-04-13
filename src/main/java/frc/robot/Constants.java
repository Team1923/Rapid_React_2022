// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.utilities.UnitConversion;

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

  public static final SupplyCurrentLimitConfiguration drivetrainCurrentLimitForTeleop =
      new SupplyCurrentLimitConfiguration(true, 60, 65, .2);

  public static final SupplyCurrentLimitConfiguration drivetrainCurrentLimitForAuton =
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
  public static final double conveyorPerent = 0.9;
  public static final double feederWheelsPercent = 0.3;

  public static final SupplyCurrentLimitConfiguration conveyorCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 30, 40, .1);

  // to be changed, RPM targets for high and low shots at fender.
  public static final double launcherRPMLowGoal =
      2400; // TOOD: Verify.  This is mathematically similar to the "working" shot we had at 1600RPM
  // * 1.5 -> 2400.
  public static final int launcherRPMHighGoal = 4050;
  public static final int launcherRPMTolerance = 35; // experimentally found.

  // pigeon ID
  public static final int pigeon = 14;

  // climber elevator spool-out prevention
  // will have some slack(not intended to be precise)
  public static final double elevatorMaxRevs = 50;
  public static final SupplyCurrentLimitConfiguration elevatorCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

  // necessary constants for autonomous

  public static final double kTrackWidthMeters = 0.73111;
  public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackWidthMeters);

  // sys-id values
  public static final double ksVolts = 0.60207;
  public static final double kvVoltsSecondsPerMeter = 0.97072;
  public static final double kaVoltSecondsSquaredPerMeter = 0.10142;
  public static final double kPDriveVel = 1.2323;

  // encoder counts per revolution
  public static final int oneRevEncodeCount = 2048;

  // wheel info
  public static final double wheelDiameter = UnitConversion.inchesToMeters(4.445 * 0.95465);
  public static final double wheelCircumeference = wheelDiameter * Math.PI;
  public static final double kGearReduction = 1 / 9.11;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // max parameters for movement
  public static final double kMaxVel = 6.6;
  public static final double kMaxAccel = 57.14;
}
