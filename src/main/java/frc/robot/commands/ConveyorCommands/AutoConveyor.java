// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ConveyorCommands;

import javax.sound.midi.SysexMessage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.utilities.UnitConversion;

public class AutoConveyor extends CommandBase {
  public ConveyorSubsystem conveyor;
  private double belts, wheels;

  private double forward_weight = 1;
  private double counter = 0;
  private boolean isReversed;

  Timer pulseTimer = new Timer();



  public AutoConveyor(
    ConveyorSubsystem con, double belts, double wheels) { // ,double frontSpeed, double backSpeed
  // Use addRequirements() here to declare subsystem dependencies.
  this.conveyor = con;
  addRequirements(conveyor);

  this.belts = belts;
  this.wheels = wheels;
  isReversed = false;


}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pulseTimer.reset();
    pulseTimer.start();

    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double deltaSeconds = pulseTimer.get() - Math.floor(pulseTimer.get());


 
      if (deltaSeconds < .25) {
        this.conveyor.runConveyor(-belts, -wheels);
        System.out.println("RUNNING FORWARDS");
      } else {
        this.conveyor.runConveyor(belts/10, wheels);
        System.out.println("RUNNING BACKWARDS");
      }
    




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pulseTimer.stop();
    conveyor.runConveyor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
