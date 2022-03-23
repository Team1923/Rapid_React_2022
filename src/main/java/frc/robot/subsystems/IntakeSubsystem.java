// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.intakeMotor);

  ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning Tab");
  ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
  ShuffleboardLayout intakeLayout =
      coachTab.getLayout("Intake", "List Layout").withPosition(3, 0).withSize(1, 5);

  public NetworkTableEntry intakeValue;
  public NetworkTableEntry coachIntakeSetpoint =
      intakeLayout
          .add("Intake Speed", 0)
          .withWidget(BuiltInWidgets.kNumberBar)
          .withSize(1, 1)
          .withPosition(0, 0)
          .getEntry();

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor.configFactoryDefault();
    intakeMotor.configSupplyCurrentLimit(Constants.intakeCurrentLimit);

    intakeMotor.setInverted(InvertType.InvertMotorOutput);

    // this will help de-noise our CAN bus ideally.
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakeValue = tuningTab.add("Intake Percentout", Constants.intakePercent).getEntry();
  }

  public void runIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
    coachIntakeSetpoint.setDouble(speed);
  }

  @Override
  public void periodic() {
    coachIntakeSetpoint.setDouble(intakeMotor.getMotorOutputPercent());
  }
}
