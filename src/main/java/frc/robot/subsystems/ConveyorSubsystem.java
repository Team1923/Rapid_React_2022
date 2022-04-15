// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.UnitConversion;

public class ConveyorSubsystem extends SubsystemBase {

  private WPI_TalonFX ConveyorMotor = new WPI_TalonFX(Constants.ConveyorMotor);
  private WPI_TalonFX FeederWheelMotor = new WPI_TalonFX(Constants.FeederWheelMotor);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  ShuffleboardTab shootingTab = Shuffleboard.getTab("Shooter Tuning Tab");

  // used for other tuning.
  public NetworkTableEntry Conveyor;
  public NetworkTableEntry FeederWheels;
  /** Creates a new CoveyorSubsystem. */
  public ConveyorSubsystem() {
    ConveyorMotor.configFactoryDefault();
    FeederWheelMotor.configFactoryDefault();

    ConveyorMotor.setInverted(InvertType.InvertMotorOutput);
    FeederWheelMotor.setInverted(InvertType.InvertMotorOutput);

    // voltage compensation will help the consistency of this mechanism a bit.
    ConveyorMotor.configVoltageCompSaturation(12);
    FeederWheelMotor.configVoltageCompSaturation(12);

    ConveyorMotor.configSupplyCurrentLimit(Constants.intakeCurrentLimit);
    FeederWheelMotor.configSupplyCurrentLimit(Constants.intakeCurrentLimit);

    // so will the current limit.

    Conveyor = tuningTab.add("Conveyor percentout", Constants.conveyorPerent).getEntry();
    FeederWheels =
        tuningTab.add("FeederWheels percentout", Constants.feederWheelsPercent).getEntry();

    ConveyorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    FeederWheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    ConveyorMotor.configNominalOutputForward(0, 30);
    ConveyorMotor.configNominalOutputReverse(0, 30);
    ConveyorMotor.configPeakOutputForward(1, 30);
    ConveyorMotor.configPeakOutputReverse(-1, 30);

    FeederWheelMotor.configNominalOutputForward(0, 30);
    FeederWheelMotor.configNominalOutputReverse(0, 30);
    FeederWheelMotor.configPeakOutputForward(1, 30);
    FeederWheelMotor.configPeakOutputReverse(-1, 30);

    ConveyorMotor.config_kP(0, 2.275, 30);
    ConveyorMotor.config_kI(0, 0, 30);
    ConveyorMotor.config_kD(0, 0, 30);
    ConveyorMotor.config_kF(0, .03, 30);

    FeederWheelMotor.config_kP(0, 2.275, 30);
    FeederWheelMotor.config_kI(0, 0, 30);
    FeederWheelMotor.config_kD(0, 0, 30);
    FeederWheelMotor.config_kF(0, 0.03, 30);
  }

  // positive is in with InvertType.None.
  // negative is out.

  public void runConveyor(double ConveyorSpd, double FeederWheelSpd) {

    ConveyorMotor.set(ControlMode.PercentOutput, ConveyorSpd);
    FeederWheelMotor.set(ControlMode.PercentOutput, FeederWheelSpd);
  }

  public void runConveyorVel(double rpm_1, double rpm_2) {
    double spd1 = UnitConversion.RPMtoNativeUnits(rpm_1);
    double spd2 = UnitConversion.RPMtoNativeUnits(rpm_2);

    ConveyorMotor.set(ControlMode.Velocity, spd1);
    FeederWheelMotor.set(ControlMode.Velocity, spd2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("Conveyor Motor: " + getConveyorVel());
    // System.out.println("Feeder Motor: " + getFeederVel());

  }

  public double getConveyorVel() {
    System.out.println("!!!");
    return UnitConversion.nativeUnitstoRPM(ConveyorMotor.getSelectedSensorVelocity());
  }

  public double getFeederVel() {
    System.out.println("???");
    return UnitConversion.nativeUnitstoRPM(FeederWheelMotor.getSelectedSensorVelocity());
  }
}
