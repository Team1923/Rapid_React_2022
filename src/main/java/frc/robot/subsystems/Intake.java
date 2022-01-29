// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Intake.RunIntakeCommand;

public class Intake extends SubsystemBase {

  private TalonFX intakeMotor = new TalonFX(Constants.intakeMotor);
  SupplyCurrentLimitConfiguration supplyCurrentLimit =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.configFactoryDefault();
    intakeMotor.configSupplyCurrentLimit(supplyCurrentLimit);
    // todo, set invert?
    // todo, change current limit.
    setDefaultCommand(new RunIntakeCommand(this));
  }

  public void runIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
