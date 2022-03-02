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

  public WPI_TalonFX ShooterWheels = new WPI_TalonFX(Constants.ShooterWheelsMotor);
  public WPI_TalonFX ShooterRollers = new WPI_TalonFX(Constants.ShooterRollersMotor);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");

  public NetworkTableEntry ShooterWheelsRPM =
      tuningTab.add("Shooter Wheels RPM", Constants.shooterWheelsRPMHighGoal).getEntry();

  public NetworkTableEntry ShooterRollersRPM =
      tuningTab.add("Shooter Rollers RPM", Constants.shooterRollerRPMHighGoal).getEntry();

  public NetworkTableEntry CURRENTShooterWheelsRPM =
      tuningTab.add("CURRENT Shooter Wheels RPM", 0).getEntry();

  public NetworkTableEntry CURRENTShooterRollersRPM =
      tuningTab.add("CURRENT Shooter Rollers RPM", 0).getEntry();

  /** Creates a new DualRollerLauncher. */
  public DualRollerLauncher() {

    ShooterWheels.configFactoryDefault();
    ShooterRollers.configFactoryDefault();

    ShooterWheels.setInverted(InvertType.InvertMotorOutput);
    ShooterRollers.setInverted(InvertType.InvertMotorOutput);

    this.ShooterRollers.setNeutralMode(NeutralMode.Coast);
    this.ShooterWheels.setNeutralMode(NeutralMode.Coast);

    ShooterWheels.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    ShooterRollers.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    ShooterWheels.configNominalOutputForward(0, 30);
    ShooterWheels.configNominalOutputReverse(0, 30);
    ShooterWheels.configPeakOutputForward(1, 30);
    ShooterWheels.configPeakOutputReverse(-1, 30);

    ShooterRollers.configNominalOutputForward(0, 30);
    ShooterRollers.configNominalOutputReverse(0, 30);
    ShooterRollers.configPeakOutputForward(1, 30);
    ShooterRollers.configPeakOutputReverse(-1, 30);

    // configure P
    this.ShooterWheels.config_kP(0, 0.25, 30);
    this.ShooterRollers.config_kP(0, 0.18, 30);

    // configure I
    this.ShooterWheels.config_kI(0, 0, 30);
    this.ShooterRollers.config_kI(0, 0, 30);

    // configure D
    this.ShooterWheels.config_kD(0, 0, 30);
    this.ShooterRollers.config_kD(0, 0, 30);

    this.ShooterWheels.config_kF(0, .055, 30);
    this.ShooterRollers.config_kF(0, .057, 30);
  }

  public void setShooterWheels() {
    double vel = UnitConversion.RPMtoNativeUnits(ShooterWheelsRPM.getDouble(0));
    ShooterWheels.set(TalonFXControlMode.Velocity, vel);
    updateRPM();
  }

  public void setShooterRollers() {
    double vel = UnitConversion.RPMtoNativeUnits(ShooterRollersRPM.getDouble(0));
    ShooterRollers.set(TalonFXControlMode.Velocity, vel);
    updateRPM();
  }
  // this is for autos.
  public void setShooterWheels(double spd) {
    ShooterWheels.set(TalonFXControlMode.Velocity, spd);
    updateRPM();
  }

  // this is for autos.
  public void setShooterRollers(double spd) {
    ShooterRollers.set(TalonFXControlMode.Velocity, spd);
    updateRPM();
  }

  public boolean inRange(double currentRPM, double targetRPM, double threshold) {
    boolean weGood = currentRPM < (targetRPM + threshold) && currentRPM > (targetRPM - threshold);
    return weGood;
  }

  public boolean ShooterWheelsInRange(double targetRPM) {
    double currentRPM = UnitConversion.nativeUnitstoRPM(ShooterWheels.getSelectedSensorVelocity());
    double target = targetRPM;
    return inRange(currentRPM, target, 75);
  }

  public boolean ShooterRollersInRange(double targetRPM) {
    double currentRPM = UnitConversion.nativeUnitstoRPM(ShooterRollers.getSelectedSensorVelocity());
    double target = targetRPM;
    return inRange(currentRPM, target, 75);
  }

  public void updateRPM() {
    CURRENTShooterWheelsRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(ShooterWheels.getSelectedSensorVelocity()));
    CURRENTShooterRollersRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(ShooterRollers.getSelectedSensorVelocity()));
  }

  public void setZero() {
    ShooterWheels.set(TalonFXControlMode.PercentOutput, 0);
    ShooterRollers.set(TalonFXControlMode.PercentOutput, 0);
  }
}
