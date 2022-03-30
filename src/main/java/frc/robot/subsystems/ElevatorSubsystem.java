// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.Servo.LinearServo;
import frc.robot.utilities.UnitConversion;
import java.util.Map;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  public WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.leftClimberMotor);

  private WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.rightClimberMotor);

  private LinearServo servo = new LinearServo(0, 50, 100);

  // tuning tab setup
  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  public NetworkTableEntry rotations = tuningTab.add("Elevator Rotations", 0).getEntry();

  // just for coach tab.
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
  ShuffleboardLayout climberLayout =
      coachTab.getLayout("Climber", "List Layout").withPosition(7, 0).withSize(1, 5);

  // for coach tab
  public NetworkTableEntry atLocation =
      climberLayout
          .add("At Limit", false)
          .withSize(1, 2)
          .withPosition(0, 0)
          .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
          .getEntry();

  public NetworkTableEntry commandedOutput =
      climberLayout.add("Elevator Percent Out", 0.0).withSize(1, 1).withPosition(0, 2).getEntry();
  public NetworkTableEntry rotationsCoach =
      climberLayout.add("Elevator Rotations", 0.0).withSize(1, 1).withPosition(0, 3).getEntry();

  public ComplexWidget rezero_encoder =
      climberLayout
          .add(
              "Re-Zero Encoder",
              new InstantCommand(
                  () -> {
                    setEncZero();
                  },
                  this))
          .withSize(1, 1)
          .withPosition(0, 5);

  public ElevatorSubsystem() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    // follow
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.InvertMotorOutput);

    leftMotor.configSupplyCurrentLimit(Constants.elevatorCurrentLimit);
    rightMotor.configSupplyCurrentLimit(Constants.elevatorCurrentLimit);

    // since we use the left motor as our "smart" one, we can turn down the encoder info from it.
    rightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    rightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

    // set up encoder logic.
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
  }

  public void runElevator(double leftSpeed, double rightSpeed) {

    if (rightSpeed > 0.1) {
      leftMotor.set(ControlMode.PercentOutput, -.5 * rightSpeed);
    }
    if (leftSpeed > 0.1) {
      leftMotor.set(ControlMode.PercentOutput, 0.8 * leftSpeed);
      runServo(40);
    }
  }

  public void runServo(double setpoint) {
    servo.setPosition(setpoint);
  }

  public void servoZero() {
    servo.setPosition(0);
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
    commandedOutput.setDouble(leftMotor.get());
    atLocation.setBoolean(overRevLimit());
  }
}
