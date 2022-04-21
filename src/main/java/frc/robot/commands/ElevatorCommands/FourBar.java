// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class FourBar extends CommandBase {
  ElevatorSubsystem elevator;
  XboxController driver;
  /** Creates a new FourBar. */
  public FourBar(ElevatorSubsystem elevator, XboxController driver) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.driver = driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.elevator.servoZero();
    // new servo logic:
    this.elevator.runServo(40);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driver.getRightBumper()) {
      // this.elevator.runServo(40);
      this.elevator.servoZero();
    } else {
      // this.elevator.servoZero();
      this.elevator.runServo(40);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this.elevator.runServo(0);
    // this.elevator.servoZero();
    this.elevator.runServo(40);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
