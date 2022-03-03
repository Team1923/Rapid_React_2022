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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.UnitConversion;
import java.util.Map;

public class DualRollerLauncher extends SubsystemBase {

  public WPI_TalonFX ShooterWheels = new WPI_TalonFX(Constants.ShooterWheelsMotor);
  public WPI_TalonFX ShooterRollers = new WPI_TalonFX(Constants.ShooterRollersMotor);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
  ShuffleboardLayout launcherLayout =
      coachTab.getLayout("Launcher", "List Layout").withPosition(6, 0).withSize(1, 5);

  /* Coach Tab Pushes */
  public NetworkTableEntry coachWheelRPM =
      launcherLayout.add("Current Wheel RPM", 0).withSize(1, 1).withPosition(0, 0).getEntry();
  public NetworkTableEntry coachRollerRPM =
      launcherLayout.add("Current Roller RPM", 0).withSize(1, 1).withPosition(0, 1).getEntry();

  public NetworkTableEntry coachWheelTargetRPM =
      launcherLayout.add("Target Wheel RPM", 0).withSize(1, 1).withPosition(0, 2).getEntry();
  public NetworkTableEntry coachRollerTargetRPM =
      launcherLayout.add("Target Roller RPM", 0).withSize(1, 1).withPosition(0, 3).getEntry();

  public NetworkTableEntry onTarget =
      launcherLayout
          .add("On Target", false)
          .withSize(1, 1)
          .withPosition(0, 4)
          .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
          .getEntry();

  /* Tuning RPM Pushes */
  public NetworkTableEntry ShooterWheelsRPM =
      tuningTab.add("Shooter Wheels RPM", Constants.shooterWheelsRPMHighGoal).getEntry();

  public NetworkTableEntry ShooterRollersRPM =
      tuningTab.add("Shooter Rollers RPM", Constants.shooterRollerRPMHighGoal).getEntry();

  public NetworkTableEntry CURRENTShooterWheelsRPM =
      tuningTab.add("CURRENT Shooter Wheels RPM", 0).getEntry();

  public NetworkTableEntry CURRENTShooterRollersRPM =
      tuningTab.add("CURRENT Shooter Rollers RPM", 0).getEntry();

  /* These two are JUST for the coach dashboard since you can't seem to get the just set
  value from shuffleboard.  Yes I'm as annoyed as you are. */

  public double wheelTargetRPM;
  public double rollerTargetRPM;

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

  /* Used to set for both auto and teleop.*/
  public void setShooterWheels(double spd) {
    ShooterWheels.set(TalonFXControlMode.Velocity, spd);
    coachWheelTargetRPM.setDouble(spd);
    wheelTargetRPM = spd;
    updateRPM();
  }
  /* Used to set for both auto and teleop.*/
  public void setShooterRollers(double spd) {
    ShooterRollers.set(TalonFXControlMode.Velocity, spd);
    coachRollerTargetRPM.setDouble(spd);
    rollerTargetRPM = spd;
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
    wheelTargetRPM = 0;
    rollerTargetRPM = 0;
  }

  @Override
  public void periodic() {
    // pushes periodic values to the coach dashboard.
    coachWheelRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(ShooterRollers.getSelectedSensorVelocity()));
    coachRollerRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(ShooterRollers.getSelectedSensorVelocity()));

    // pushes the "we good" boolean to the coach dashboard.

    onTarget.setBoolean(
        wheelTargetRPM != 0
            && (ShooterWheelsInRange(wheelTargetRPM) && ShooterRollersInRange(rollerTargetRPM)));
  }
}
