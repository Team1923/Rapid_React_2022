// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.UnitConversion;

public class DualRollerLauncher extends SubsystemBase {

  public WPI_TalonFX frontMotor = new WPI_TalonFX(Constants.frontRollerMotor);
  public WPI_TalonFX backMotor = new WPI_TalonFX(Constants.backRollerMotor);

  ShuffleboardTab tuneDualRollerTab = Shuffleboard.getTab("tune dual roller");

  public NetworkTableEntry front_setpt =
      tuneDualRollerTab.add("Launcher front set point", 0).getEntry();
  public NetworkTableEntry front_kp = tuneDualRollerTab.add("Launcher front P value", 0).getEntry();
  public NetworkTableEntry front_ki = tuneDualRollerTab.add("Launcher front I value", 0).getEntry();
  public NetworkTableEntry front_kd = tuneDualRollerTab.add("Launcher front D value", 0).getEntry();
  double front_integral, front_error, front_derivative, front_previous_error, front_velocity = 0;
  public NetworkTableEntry frontRPM = tuneDualRollerTab.add("Front RPM", 0).getEntry();

  public NetworkTableEntry back_setpt =
      tuneDualRollerTab.add("Launcher back set point", 0).getEntry();
  public NetworkTableEntry back_kp = tuneDualRollerTab.add("Launcher back P value", 0).getEntry();
  public NetworkTableEntry back_ki = tuneDualRollerTab.add("Launcher back I value", 0).getEntry();
  public NetworkTableEntry back_kd = tuneDualRollerTab.add("Launcher back D value", 0).getEntry();
  double back_integral, back_error, back_derivative, back_previous_error, back_velocity = 0;
  public NetworkTableEntry backRPM = tuneDualRollerTab.add("Back RPM", 0).getEntry();

  /** Creates a new DualRollerLauncher. */
  public DualRollerLauncher() {

    // this is for in the event the robot reboots we need to explicitly set configurations
    // to avoid a latent state breaking stuff.

    frontMotor.configFactoryDefault();
    backMotor.configFactoryDefault();

    frontMotor.setInverted(InvertType.InvertMotorOutput);
    backMotor.setInverted(InvertType.InvertMotorOutput);

    this.backMotor.setNeutralMode(NeutralMode.Coast);
    this.frontMotor.setNeutralMode(NeutralMode.Coast);

    frontMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    backMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    // copies the output range of (-1,1) from the examples, a nominal output of zero.
    frontMotor.configNominalOutputForward(0, 30);
    frontMotor.configNominalOutputReverse(0, 30);
    frontMotor.configPeakOutputForward(1, 30);
    frontMotor.configPeakOutputReverse(-1, 30);

    backMotor.configNominalOutputForward(0, 30);
    backMotor.configNominalOutputReverse(0, 30);
    backMotor.configPeakOutputForward(1, 30);
    backMotor.configPeakOutputReverse(-1, 30);

    // configure P
    this.frontMotor.config_kP(0, 0.25, 30);
    this.backMotor.config_kP(0, 0.18, 30);

    // configure I
    this.frontMotor.config_kI(0, 0, 30);
    this.backMotor.config_kI(0, 0, 30);

    // configure D
    this.frontMotor.config_kD(0, 0, 30);
    this.backMotor.config_kD(0, 0, 30);

    this.frontMotor.config_kF(0, .05, 30);
    this.backMotor.config_kF(0, .057, 30);
  }

  public void setFront() {
    double vel = UnitConversion.nativeUnitstoRPM(frontRPM.getDouble(0));
    frontMotor.set(TalonFXControlMode.Velocity, vel);
  }

  public boolean inRange(double currentRPM, double targetRPM, double threshold) {
    boolean weGood = currentRPM < (targetRPM + threshold) && currentRPM > (targetRPM - threshold);
    return weGood;
  }

  public boolean frontInRange() {
    double currentRPM = UnitConversion.nativeUnitstoRPM(frontMotor.getSelectedSensorVelocity());
    double target = frontRPM.getDouble(0);
    return inRange(currentRPM, target, 50);
  }

  public boolean backInRange() {
    double currentRPM = UnitConversion.nativeUnitstoRPM(backMotor.getSelectedSensorVelocity());
    double target = backRPM.getDouble(0);
    return inRange(currentRPM, target, 50);
  }

  public void setBack() {
    double vel = UnitConversion.RPMtoNativeUnits(backRPM.getDouble(0));
    backMotor.set(TalonFXControlMode.Velocity, vel);
  }

  public void setZero() {
    frontMotor.set(TalonFXControlMode.PercentOutput, 0);
    backMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
}
