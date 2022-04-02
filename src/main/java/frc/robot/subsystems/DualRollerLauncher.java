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
import frc.robot.utilities.RollingAvgDouble;
import frc.robot.utilities.UnitConversion;
import java.util.Map;

public class DualRollerLauncher extends SubsystemBase {

  public WPI_TalonFX launcherMotorA = new WPI_TalonFX(Constants.WheelMotorCanIDA);
  public WPI_TalonFX launcherMotorB = new WPI_TalonFX(Constants.WheelMotorCANIDB);

  public RollingAvgDouble rollingRPMAvg = new RollingAvgDouble(.5);

  // shuffleboard coachDashboard
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
  ShuffleboardLayout launcherLayout =
      coachTab.getLayout("Launcher", "List Layout").withPosition(6, 0).withSize(1, 5);

  /* Coach Tab Pushes */
  public NetworkTableEntry coachShooterRPM =
      launcherLayout.add("Current Shooter RPM", 0).withSize(1, 1).withPosition(0, 0).getEntry();

  public NetworkTableEntry coachShooterTargetRPM =
      launcherLayout
          .add("Target Shooter RPM", Constants.launcherRPMHighGoal)
          .withSize(1, 1)
          .withPosition(0, 2)
          .getEntry();

  public NetworkTableEntry onTarget =
      launcherLayout
          .add("On Target", false)
          .withSize(1, 1)
          .withPosition(0, 4)
          .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
          .getEntry();
  
  
  public NetworkTableEntry isSpinning =
          launcherLayout
              .add("SPINNING:", false)
              .withSize(1, 1)
              .withPosition(0, 8)
              .withProperties(Map.of("Color when false", "#000000", "Color when true", "#17FC03"))
              .getEntry();

  public double shooterTargetRPM;

  /** Creates a new DualRollerLauncher. */
  public DualRollerLauncher() {

    launcherMotorA.configFactoryDefault();
    launcherMotorB.configFactoryDefault();

    launcherMotorB.follow(launcherMotorA);

    this.launcherMotorB.setNeutralMode(NeutralMode.Coast);
    this.launcherMotorA.setNeutralMode(NeutralMode.Coast);

    launcherMotorA.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    launcherMotorB.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    launcherMotorA.configNominalOutputForward(0, 30);
    launcherMotorA.configNominalOutputReverse(0, 30);
    launcherMotorA.configPeakOutputForward(1, 30);
    launcherMotorA.configPeakOutputReverse(-1, 30);

    launcherMotorA.configVoltageCompSaturation(12);
    launcherMotorB.configVoltageCompSaturation(12);

    this.launcherMotorA.config_kP(0, .275, 30);
    this.launcherMotorA.config_kI(0, .00012, 30);
    this.launcherMotorA.config_kD(0, 0, 30);
    this.launcherMotorA.config_kF(0, .03, 30);

    launcherMotorA.setInverted(InvertType.None); // can change this without changing below logic.
    launcherMotorB.setInverted(InvertType.OpposeMaster);
  }

  public void setLauncherSpeedRPM(double spd) {
    spd = UnitConversion.nativeUnitstoRPM(spd);
    setLauncherSpeedCTR(spd);
  }

  /* Used to set for both auto and teleop.*/
  public void setLauncherSpeedCTR(double spd) {
    launcherMotorA.set(TalonFXControlMode.Velocity, spd);
    shooterTargetRPM = spd;
  }

  public boolean inRange(double currentRPM, double targetRPM, double threshold) {
    boolean weGood = currentRPM < (targetRPM + threshold) && currentRPM > (targetRPM - threshold);
    return weGood;
  }

  public boolean launcherInRange(double targetRPM) {
    double currentRPM = UnitConversion.nativeUnitstoRPM(launcherMotorA.getSelectedSensorVelocity());
    double target = targetRPM;
    return inRange(currentRPM, target, Constants.launcherRPMTolerance);
  }

  public void setZero() {
    launcherMotorA.set(TalonFXControlMode.PercentOutput, 0);
    launcherMotorB.set(TalonFXControlMode.PercentOutput, 0);
    shooterTargetRPM = 0;
  }

  public void pidShuffleboard() {
    this.launcherMotorA.setIntegralAccumulator(
        0); // this is used to reset the integral value when we try again so there's no extra windup
    // left-over from prior uses.
  }

  @Override
  public void periodic() {
    this.rollingRPMAvg.add(
        UnitConversion.nativeUnitstoRPM(launcherMotorA.getSelectedSensorVelocity()));

    // coach dashboard stuff
    coachShooterRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(launcherMotorA.getSelectedSensorVelocity()));
    if ((launcherInRange(Constants.launcherRPMHighGoal))
        || (launcherInRange(Constants.launcherRPMLowGoal))) {
      onTarget.setBoolean(true);
    } else {
      onTarget.setBoolean(false);
    }

    if(launcherMotorA.getMotorOutputVoltage() > 0.5){
      isSpinning.setBoolean(true);
    }
    else{
      isSpinning.setBoolean(false);
    }
  }
}
