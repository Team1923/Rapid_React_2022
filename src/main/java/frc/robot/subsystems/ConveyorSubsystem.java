// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {

  private TalonFX ConveyorMotor = new TalonFX(Constants.ConveyorMotor);
  private TalonFX FeederWheelMotor = new TalonFX(Constants.FeederWheelMotor);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  ShuffleboardTab shootingTab = Shuffleboard.getTab("Shooter Tuning Tab");
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
  ShuffleboardLayout intakeLayout =
      coachTab.getLayout("Feeder + Conveyor", "List Layout").withPosition(4, 0).withSize(2, 5);
  public NetworkTableEntry shotCount = shootingTab.add("Shot Count", 0).getEntry();
  // just for the coach dashboard.
  public NetworkTableEntry coachConveyor =
      intakeLayout
          .add("Conveyor Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(1, 1)
          .withPosition(0, 0)
          .getEntry();

  public NetworkTableEntry coachFeeder =
      intakeLayout
          .add("Feeder Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(1, 1)
          .withPosition(0, 1)
          .getEntry();

  // used for other tuning.
  public NetworkTableEntry Conveyor;
  public NetworkTableEntry FeederWheels;
  /** Creates a new CoveyorSubsystem. */
  public ConveyorSubsystem() {
    ConveyorMotor.configFactoryDefault();
    FeederWheelMotor.configFactoryDefault();

    ConveyorMotor.setInverted(InvertType.None);
    FeederWheelMotor.setInverted(InvertType.None);

    // voltage compensation will help the consistency of this mechanism a bit.
    ConveyorMotor.configVoltageCompSaturation(12);
    FeederWheelMotor.configVoltageCompSaturation(12);

    // so will the current limit.

    Conveyor = tuningTab.add("Conveyor percentout", Constants.conveyorPerent).getEntry();
    FeederWheels =
        tuningTab.add("FeederWheels percentout", Constants.feederWheelsPercent).getEntry();
  }

  // positive is in with InvertType.None.
  // negative is out.

  public void runConveyor(double ConveyorSpd, double FeederWheelSpd) {
    ConveyorMotor.set(ControlMode.PercentOutput, ConveyorSpd);
    FeederWheelMotor.set(ControlMode.PercentOutput, FeederWheelSpd);

    coachConveyor.setDouble(ConveyorSpd);
    coachFeeder.setDouble(FeederWheelSpd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    coachConveyor.setDouble(0);
    coachFeeder.setDouble(0);
  }
}
