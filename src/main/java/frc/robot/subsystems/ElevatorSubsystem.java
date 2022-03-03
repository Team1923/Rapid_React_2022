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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.UnitConversion;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.leftClimberMotor);

  private WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.rightClimberMotor);

  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
  ShuffleboardLayout climberLayout =
      coachTab.getLayout("Climber", "List Layout").withPosition(0, 0).withSize(1, 5);

  public NetworkTableEntry rotations = tuningTab.add("Elevator Rotations", 0).getEntry();
  public ComplexWidget t =
      tuningTab.add(
          "re-zero encoder",
          new InstantCommand(
              () -> {
                setEncZero();
              },
              this));

  public NetworkTableEntry atLocation =
      climberLayout.add("At Limit", false).withSize(1, 1).withPosition(0, 0).getEntry();
  public NetworkTableEntry commandedOutput =
      climberLayout.add("Elevator %Out", 0).withSize(1, 1).withPosition(0, 1).getEntry();
  public NetworkTableEntry rotationsCoach =
      climberLayout.add("Elevator Rotations", 0).withSize(1, 1).withPosition(0, 2).getEntry();

  public ElevatorSubsystem() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    // follow
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.InvertMotorOutput);

    leftMotor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    rightMotor.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

    // set up encoder logic.
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    // configing motion magic
    leftMotor.selectProfileSlot(0, 0);
    leftMotor.config_kF(Constants.kIdx, Constants.kF, Constants.kTimeoutMs);
    leftMotor.config_kP(Constants.kIdx, Constants.kP, Constants.kTimeoutMs);
    leftMotor.config_kI(Constants.kIdx, Constants.kI, Constants.kTimeoutMs);
    leftMotor.config_kD(Constants.kIdx, Constants.kD, Constants.kTimeoutMs);

    // setDefaultCommand(new ElevatorKeepDown(this));
  }

  public void runElevator(double leftSpeed, double rightSpeed) {
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

  public double encVal() {
    return -1 * leftMotor.getSelectedSensorPosition(0);
  }

  public boolean overRevLimit() {
    return UnitConversion.positionNativeToRots(encVal()) > Constants.elevatorMaxRevs;
  }

  public void setEncZero() {
    System.out.println("Zeroed Elevator");
    leftMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotations.setDouble(UnitConversion.positionNativeToRots(encVal()));
    rotationsCoach.setDouble(UnitConversion.positionNativeToRots(encVal()));
    commandedOutput.setDouble(leftMotor.getMotorOutputVoltage()); // should be output volts?
    atLocation.setBoolean(overRevLimit());
  }
}
