// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.intakeMotor, Constants.canivoreBus);

  SupplyCurrentLimitConfiguration supplyCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

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
    intakeMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    // todo, set invert?
    // todo, change current limit.

    intakeMotor.setInverted(InvertType.InvertMotorOutput);

    intakeValue = tuningTab.add("Intake Percentout", Constants.intakePercent).getEntry();

    // The default command may want to be running.
    // Not 100% sure if we want that or not, until then it'll be disabled.

    // setDefaultCommand(new RunIntakeCommand(this, 0));
  }

  public void runIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
    coachIntakeSetpoint.setDouble(speed);
  }

  @Override
  public void periodic() {
    coachIntakeSetpoint.setDouble(0);
  }
}
