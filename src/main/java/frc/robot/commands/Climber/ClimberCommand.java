// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  /** Creates a new ClimberCommand. */

  boolean isFinished = false;
  public ClimberCommand(ClimberSubsystem climber, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    climber.runClimber(speed);
         
    TalonFX leftMotor = climber.getLeftMotor();

    double currentValue = leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30).value;
    isFinished = climber.isInRange(currentValue, 10, 0.2);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
