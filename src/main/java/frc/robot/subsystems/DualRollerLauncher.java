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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.RollingAvgDouble;
import frc.robot.utilities.UnitConversion;

public class DualRollerLauncher extends SubsystemBase {

  public WPI_TalonFX launcherMotorA = new WPI_TalonFX(Constants.WheelMotorCanIDA);
  public WPI_TalonFX launcherMotorB = new WPI_TalonFX(Constants.WheelMotorCANIDB);

  ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter Tuning Tab");

  public RollingAvgDouble rollingRPMAvg = new RollingAvgDouble(.5);

  /* Tuning RPM Pushes */
  public NetworkTableEntry rpm =
      shooterTab.add("Target RPM", Constants.launcherRPMHighGoal).getEntry();

  public NetworkTableEntry PValue = shooterTab.add("P VALUE", 0.275).getEntry();
  public NetworkTableEntry IValue = shooterTab.add("I VALUE", 0.00012).getEntry();
  public NetworkTableEntry DValue = shooterTab.add("D VALUE", 0).getEntry();
  public NetworkTableEntry FFValue = shooterTab.add("Feed Forward", 0.03).getEntry();

  public NetworkTableEntry CURRENTShooterWheelsRPM =
      shooterTab
          .add("CURRENT WHEEL RPM", 0)
          .withPosition(0, 0)
          .withSize(6, 3)
          .withWidget(BuiltInWidgets.kGraph)
          .getEntry();

  public double wheelTargetRPM;

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
    wheelTargetRPM = spd;
    updateRPM();
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

  public void updateRPM() {
    CURRENTShooterWheelsRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(launcherMotorA.getSelectedSensorVelocity()));
  }

  public void setZero() {
    launcherMotorA.set(TalonFXControlMode.PercentOutput, 0);
    launcherMotorB.set(TalonFXControlMode.PercentOutput, 0);
    wheelTargetRPM = 0;
    CURRENTShooterWheelsRPM.setDouble(0);
  }

  public void pidShuffleboard() {
    this.launcherMotorA.config_kP(0, PValue.getDouble(0), 30); // .25
    this.launcherMotorA.config_kI(0, IValue.getDouble(0), 30); // .0001 per testing?
    this.launcherMotorA.config_kD(0, DValue.getDouble(0), 30);

    this.launcherMotorA.setIntegralAccumulator(
        0); // this is used to reset the integral value when we try again so there's no extra windup
    // left-over from prior uses.
  }

  @Override
  public void periodic() {
    this.rollingRPMAvg.add(
        UnitConversion.nativeUnitstoRPM(launcherMotorA.getSelectedSensorVelocity()));
  }
}
