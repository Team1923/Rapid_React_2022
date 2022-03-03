package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;

public class DriveTrainSubsystem extends SubsystemBase {
  private WPI_TalonFX r1 = new WPI_TalonFX(Constants.r1);
  private WPI_TalonFX r2 = new WPI_TalonFX(Constants.r2);
  private WPI_TalonFX r3 = new WPI_TalonFX(Constants.r3);
  private WPI_TalonFX l1 = new WPI_TalonFX(Constants.l1);
  private WPI_TalonFX l2 = new WPI_TalonFX(Constants.l2);
  private WPI_TalonFX l3 = new WPI_TalonFX(Constants.l3);

  public NetworkTableEntry driveLVolts;
  public NetworkTableEntry driveRVolts;

  public DifferentialDrive kDrive = new DifferentialDrive(l1, r1);

  SupplyCurrentLimitConfiguration supplyCurrentLimitConfiguration =
      new SupplyCurrentLimitConfiguration(true, 60, 65, 3);

  public DriveTrainSubsystem() {
    r1.configFactoryDefault();
    r2.configFactoryDefault();
    r3.configFactoryDefault();
    l1.configFactoryDefault();
    l2.configFactoryDefault();
    l3.configFactoryDefault();

    r1.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);
    l1.configSupplyCurrentLimit(supplyCurrentLimitConfiguration);

    r1.setNeutralMode(NeutralMode.Coast);
    l1.setNeutralMode(NeutralMode.Coast);
    r2.setNeutralMode(NeutralMode.Coast);
    l2.setNeutralMode(NeutralMode.Coast);
    r3.setNeutralMode(NeutralMode.Brake);
    l3.setNeutralMode(NeutralMode.Brake);

    r2.follow(r1);
    r3.follow(r1);
    l2.follow(l1);
    l3.follow(l1);

    l2.setInverted(InvertType.FollowMaster);
    l3.setInverted(InvertType.FollowMaster);
    l1.setInverted(InvertType.InvertMotorOutput);

    ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
    ShuffleboardLayout driveLayout =
        coachTab.getLayout("Drivetrain", "List Layout").withPosition(2, 0).withSize(2, 5);

    driveLVolts =
        driveLayout
            .add("LVolts", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withSize(2, 1)
            .withPosition(0, 0)
            .withProperties(Map.of("Min", -12, "Max", 12, "Title", "Leftside Volts"))
            .getEntry();
    driveRVolts =
        driveLayout
            .add("RVolts", 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withSize(2, 1)
            .withPosition(0, 1)
            .withProperties(Map.of("Min", -12, "Max", 12, "Title", "Rightside Volts"))
            .getEntry();

    setDefaultCommand(
        new RunCommand(
            () -> {
              kDrive.feed();
            },
            this));
  }

  @Override
  public void periodic() {
    driveLVolts.setDouble(l1.getMotorOutputVoltage());
    driveRVolts.setDouble(r1.getMotorOutputVoltage());
  }
}
