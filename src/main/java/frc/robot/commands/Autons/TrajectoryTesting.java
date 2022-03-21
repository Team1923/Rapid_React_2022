// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveTrainCommands.AutoDrive;
import frc.robot.commands.IntakeCommands.AutoIntake;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DualRollerLauncher;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryTesting extends SequentialCommandGroup {
  IntakeSubsystem intake;
  DualRollerLauncher drl;
  DriveTrainSubsystem drive;
  ConveyorSubsystem conveyor;
  /** Creates a new TwoBallHighAuto. */
  public TrajectoryTesting(
    IntakeSubsystem intake,
    DualRollerLauncher drl,
    DriveTrainSubsystem drive,
    ConveyorSubsystem conveyor

  ) {
    this.intake = intake;
    this.drl = drl;
    this.drive = drive;
    this.conveyor = conveyor;
  }

  //basically setup a constraint on voltage so you don't go too fast
   DifferentialDriveVoltageConstraint autoVoltageConstraint = 
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA), Constants.kDrive, 12);

  //configure the actual trajectory
  TrajectoryConfig config = 
    new TrajectoryConfig(Constants.kMaxVel, Constants.kMaxAccel).setKinematics(Constants.kDrive).addConstraint(autoVoltageConstraint);

  //get the actual trajectory from PathWeaver
  String file = "/PathWeaver/Paths/PickUp3and4.path";
  Trajectory pickUp = new Trajectory();

  try{
    Path path = Filesystem.getDeployDirectory().toPath().resolve(file); 
    pickUp = TrajectoryUtil.fromPathweaverJson(path);
  }
  catch(IOException ex){
    System.out.println("Unable to open trajectory");
  }
//need to fix this command
  RamseteCommand ramseteCommand = 
    new RamseteCommand(
      pickUp, 
     ()-> {},  //pose
     new RamseteController(Constants.KRamseteB, Constants.kRamseteZeta), 
     new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kDrive), 
     Constants.kDrive,
     ()-> {}, 
     new PIDController(Constants.kP, 0, 0), 
     new PIDController(Constants.kP, 0, 0),
      () ->{},
      drive);
}
}

