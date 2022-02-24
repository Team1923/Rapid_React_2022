// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {

  private TalonFX ConveyorMotor = new TalonFX(Constants.ConveyorMotor);
  private TalonFX FeederWheelMotor = new TalonFX(Constants.FeederWheelMoter);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");

  public NetworkTableEntry Conveyor;
  public NetworkTableEntry FeederWheels;
  /** Creates a new CoveyorSubsystem. */
  public ConveyorSubsystem() {
    ConveyorMotor.configFactoryDefault();
    FeederWheelMotor.configFactoryDefault();
    ConveyorMotor.setInverted(InvertType.InvertMotorOutput);
    FeederWheelMotor.setInverted(InvertType.InvertMotorOutput);

    Conveyor = tuningTab.add("Conveyor percentout", 0).getEntry();
    FeederWheels = tuningTab.add("FeederWheels percentout", 0).getEntry();
  }

  public void runConveyor(double ConveyorSpd, double FeederWheelSpd) {
    ConveyorMotor.set(ControlMode.PercentOutput, ConveyorSpd);
    FeederWheelMotor.set(ControlMode.PercentOutput, FeederWheelSpd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
