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

  private TalonFX conveyor = new TalonFX(Constants.conveyor);
  private TalonFX feeder = new TalonFX(Constants.feeder);

  ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

  public NetworkTableEntry belts;
  public NetworkTableEntry wheels;
  /** Creates a new CoveyorSubsystem. */
  public ConveyorSubsystem() {
    conveyor.configFactoryDefault();
    feeder.configFactoryDefault();
    conveyor.setInverted(InvertType.InvertMotorOutput);
    feeder.setInverted(InvertType.InvertMotorOutput);

    belts = tuneDualRollerTab.add("Belts percentout", 0).getEntry();
    wheels = tuneDualRollerTab.add("Wheels percentout", 0).getEntry();
  }

  public void runConveyor(double backSpeed, double frontSpeed) {
    conveyor.set(ControlMode.PercentOutput, backSpeed);
    feeder.set(ControlMode.PercentOutput, frontSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
