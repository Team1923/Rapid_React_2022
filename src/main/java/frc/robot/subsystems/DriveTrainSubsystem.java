package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  private WPI_TalonFX r1 = new WPI_TalonFX(Constants.r1);
  private WPI_TalonFX r2 = new WPI_TalonFX(Constants.r2);
  private WPI_TalonFX r3 = new WPI_TalonFX(Constants.r3);
  private WPI_TalonFX l1 = new WPI_TalonFX(Constants.l1);
  private WPI_TalonFX l2 = new WPI_TalonFX(Constants.l2);
  private WPI_TalonFX l3 = new WPI_TalonFX(Constants.l3);

  public DifferentialDrive kDrive = new DifferentialDrive(l1, r1);

  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

  public DriveTrainSubsystem() {

    // follower setup

    // Taken from example here:
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/DifferentialDrive/src/main/java/frc/robot/Robot.java#L143

    r1.configFactoryDefault();
    r2.configFactoryDefault();
    r3.configFactoryDefault();
    l1.configFactoryDefault();
    l2.configFactoryDefault();
    l3.configFactoryDefault();

    r1.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    l1.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

    // r1.setNeutralMode(NeutralMode.Brake);
    // l1.setNeutralMode(NeutralMode.Brake);

    r2.follow(r1);
    r3.follow(r1);
    l2.follow(l1);
    l3.follow(l1);

    // set inverts to make "full forward" actually make the robot go forward.

    l2.setInverted(InvertType.FollowMaster);
    l3.setInverted(InvertType.FollowMaster);
    l1.setInverted(InvertType.InvertMotorOutput);

    // setDefaultCommand(new StopDT(this));
  }

  public void setSpeed(double rightSpeed, double leftSpeed) {

    r1.set(ControlMode.PercentOutput, rightSpeed);
    l1.set(ControlMode.PercentOutput, leftSpeed);
  }

  // arcade drive function to the subsystem, not the command.  called with an external command.
  // taken from 3847:
  // https://github.com/Spectrum3847/Ultraviolet-2020/blob/master/src/main/java/frc/robot/subsystems/Drivetrain.java#L128

}
