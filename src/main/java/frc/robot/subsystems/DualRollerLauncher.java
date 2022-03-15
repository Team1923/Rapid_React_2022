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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.UnitConversion;
import java.util.Map;

public class DualRollerLauncher extends SubsystemBase {

  public WPI_TalonFX ShooterWheelA = new WPI_TalonFX(Constants.ShooterMotorA);
  public WPI_TalonFX ShooterWheelB = new WPI_TalonFX(Constants.ShooterMotorB);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");

  // stubs for later use.
  ShuffleboardTab shooterTuneTab = Shuffleboard.getTab("Tune Launcher");
  ShuffleboardLayout wheelLauncherLayout, launcherTuneLayout;
  NetworkTableEntry wheelkP, wheelkI, wheelkD, wheelkFF, wheelTargetRPMTuning, wheelCurrentRPM;

  // for coach tab
  ShuffleboardLayout launcherLayout =
      coachTab.getLayout("Launcher", "List Layout").withPosition(6, 0).withSize(1, 5);

  /* Coach Tab Pushes */
  public NetworkTableEntry coachWheelRPM =
      launcherLayout.add("Current Wheel RPM", 0).withSize(1, 1).withPosition(0, 0).getEntry();

  public NetworkTableEntry coachWheelTargetRPM =
      launcherLayout
          .add("Target Wheel RPM", Constants.shooterWheelsRPMHighGoal)
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

  /* Tuning RPM Pushes */
  public NetworkTableEntry ShooterWheelsRPM = tuningTab.add("Shooter Wheels RPM", 0).getEntry();

  public NetworkTableEntry CURRENTShooterWheelsRPM =
      tuningTab.add("CURRENT Shooter Wheels RPM", 0).getEntry();

  /* These two are JUST for the coach dashboard since you can't seem to get the just set
  value from shuffleboard.  Yes I'm as annoyed as you are. */

  public double wheelTargetRPM;

  /** Creates a new DualRollerLauncher. */
  public DualRollerLauncher() {

    ShooterWheelA.configFactoryDefault();
    ShooterWheelB.configFactoryDefault();

    ShooterWheelB.follow(ShooterWheelA); // since they're mechanically linked this is fine.

    ShooterWheelA.setInverted(InvertType.InvertMotorOutput);
    ShooterWheelB.setInverted(
        InvertType.OpposeMaster); // as this is opposite side this *should* still work.

    // both are explicitly set but that's *fine*.
    this.ShooterWheelB.setNeutralMode(NeutralMode.Coast);
    this.ShooterWheelA.setNeutralMode(NeutralMode.Coast);

    ShooterWheelA.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    ShooterWheelB.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    ShooterWheelA.configNominalOutputForward(0, 30);
    ShooterWheelA.configNominalOutputReverse(0, 30);
    ShooterWheelA.configPeakOutputForward(1, 30);
    ShooterWheelA.configPeakOutputReverse(-1, 30);

    // retune needed.
    this.ShooterWheelA.config_kP(0, 0.25, 30);
    this.ShooterWheelA.config_kI(0, 0, 30);
    this.ShooterWheelA.config_kD(0, 0, 30);
    this.ShooterWheelA.config_kF(0, .055, 30);

    if (Constants.tuning) {
      this.launcherTuneLayout =
          shooterTuneTab.getLayout("System Info", "List Layout").withPosition(6, 0).withSize(2, 6);

      // wheel layout setup.
      // important to note that certain types of properties (ie position) do not work.
      this.wheelCurrentRPM =
          this.shooterTuneTab
              .add("Curr. Wheel RPM", 0)
              .withPosition(0, 0)
              .withSize(6, 3)
              .withWidget(BuiltInWidgets.kGraph)
              .withProperties(Map.of("Visible time", 60))
              .getEntry();

      this.wheelkP = shooterTuneTab.add("Wheel kP", 0).withPosition(0, 3).withSize(3, 1).getEntry();
      this.wheelkI = shooterTuneTab.add("Wheel kI", 0).withPosition(3, 3).withSize(3, 1).getEntry();
      this.wheelkD = shooterTuneTab.add("Wheel kD", 0).withPosition(0, 4).withSize(3, 1).getEntry();
      this.wheelkFF =
          shooterTuneTab.add("Wheel kFF", 0).withPosition(3, 4).withSize(3, 1).getEntry();

      this.wheelTargetRPMTuning =
          launcherTuneLayout
              .add("Tar. Wheel RPM", Constants.shooterWheelsRPMHighGoal)
              .withSize(1, 2)
              .withPosition(0, 0)
              .getEntry();

      // command to set PID from dash & run in-line.  may work, may not.  tbd.
      this.launcherTuneLayout.add(
          new SequentialCommandGroup(
              new RunCommand(
                  () -> {
                    // wheel config.
                    this.ShooterWheelA.config_kP(0, this.wheelkP.getDouble(0), 30);
                    this.ShooterWheelA.config_kI(0, this.wheelkI.getDouble(0), 30);
                    this.ShooterWheelA.config_kD(0, this.wheelkD.getDouble(0), 30);
                    this.ShooterWheelA.config_kF(0, this.wheelkFF.getDouble(0), 30);
                  },
                  this),
              new RunCommand(
                  () -> {
                    this.setShooterWheels((this.wheelTargetRPMTuning.getDouble(0)));
                  },
                  this)));
    }
  }

  /* Used to set for both auto and teleop.*/
  public void setShooterWheels(double spd) {
    ShooterWheelA.set(TalonFXControlMode.Velocity, spd);
    // coachWheelTargetRPM.setDouble(spd);
    wheelTargetRPM = spd;
    updateRPM();
  }

  public boolean inRange(double currentRPM, double targetRPM, double threshold) {
    boolean weGood = currentRPM < (targetRPM + threshold) && currentRPM > (targetRPM - threshold);
    return weGood;
  }

  public boolean ShooterWheelsInRange(double targetRPM) {
    double currentRPM = UnitConversion.nativeUnitstoRPM(ShooterWheelA.getSelectedSensorVelocity());
    double target = targetRPM;
    return inRange(currentRPM, target, 75);
  }

  public void updateRPM() {
    CURRENTShooterWheelsRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(ShooterWheelA.getSelectedSensorVelocity()));
  }

  public void setZero() {
    ShooterWheelA.set(TalonFXControlMode.PercentOutput, 0);
    ShooterWheelB.set(TalonFXControlMode.PercentOutput, 0);
    wheelTargetRPM = 0;
  }

  @Override
  public void periodic() {
    // pushes periodic values to the coach dashboard.
    coachWheelRPM.setDouble(
        UnitConversion.nativeUnitstoRPM(ShooterWheelA.getSelectedSensorVelocity()));
    // pushes the "we good" boolean to the coach dashboard.

    if (Constants.tuning) {
      this.wheelCurrentRPM.setDouble(
          UnitConversion.nativeUnitstoRPM(ShooterWheelA.getSelectedSensorVelocity()));
    }

    if (ShooterWheelsInRange(Constants.shooterWheelsRPMHighGoal)
        || (ShooterWheelsInRange(Constants.shooterWheelsRPMLowGoal))) {
      onTarget.setBoolean(true);
    } else {
      onTarget.setBoolean(false);
    }
  }
}
