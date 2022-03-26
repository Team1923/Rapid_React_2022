package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.UnitConversion;
import frc.robot.utilities.can.ctre.status_frame.StatusFrameHelper;
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

  private DifferentialDriveOdometry m_odometry;
  public static PigeonIMU gyro = new PigeonIMU(Constants.pigeon);

  private final double kEncoderTick2Meter = 1.0 / 2048 * 1.0 / 9.1 * Math.PI * 0.127;



  // getting the necessary conversion factor to convert for velocity

  PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
  PigeonIMU.GeneralStatus generalStatus = new PigeonIMU.GeneralStatus();

  public DriveTrainSubsystem() {

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    resetEncoders();

    gyro.setFusedHeading(0);

    r1.configFactoryDefault();
    r2.configFactoryDefault();
    r3.configFactoryDefault();
    l1.configFactoryDefault();
    l2.configFactoryDefault();
    l3.configFactoryDefault();

    r1.configSupplyCurrentLimit(Constants.drivetrainCurrentLimit);
    l1.configSupplyCurrentLimit(Constants.drivetrainCurrentLimit);

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

    l1.setInverted(InvertType.InvertMotorOutput);
    l2.setInverted(InvertType.FollowMaster);
    l3.setInverted(InvertType.FollowMaster);

    // this denoises our followers so we don't have to worry about the CANbus being as noisy.
    // if this causes _any_ issues comment it out and we'll be fine.
    StatusFrameHelper.setStatusFrame(StatusFrame.Status_1_General, 255, l2, l3, r2, r3);
    StatusFrameHelper.setStatusFrame(StatusFrame.Status_2_Feedback0, 255, l2, l3, r2, r3);

    ShuffleboardTab coachTab = Shuffleboard.getTab("Coach Dashboard");
    ShuffleboardLayout driveLayout =
        coachTab.getLayout("Drivetrain", "List Layout").withPosition(1, 0).withSize(2, 5);

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
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
    // System.out.println("Speed: " + UnitConversion.tickSecTomSec(r1.getSelectedSensorVelocity()));
    //System.out.println("Distance: " + getLeftDistance());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double encodeticks() {
    return l1.getSelectedSensorPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeed() {
    System.out.println("ACTUAL DISTANCE: " + getRightDistance());
    return new DifferentialDriveWheelSpeeds(
        UnitConversion.tickSecTomSec(
            l1.getSelectedSensorVelocity()),
        UnitConversion.tickSecTomSec(
            r1.getSelectedSensorVelocity()));

    // l1.getSelectedSensorVelocity() * kEncoderTick2Meter,
    // r1.getSelectedSensorVelocity() * kEncoderTick2Meter);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    System.out.println("Stopping the auton command");
    l1.setVoltage(leftVolts);
    r1.setVoltage(rightVolts);


    kDrive.feed();
  }

  public void tankDriveVelocity(double leftVel, double rightVel) {}

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void resetEncoders() {
    l1.getSensorCollection().setIntegratedSensorPosition(0, 10);
    l2.getSensorCollection().setIntegratedSensorPosition(0, 10);
  }

  public double getLeftDistance(){
    return 1.0 * l1.getSelectedSensorPosition() * kEncoderTick2Meter;

    //return 1.0 *l1.getSelectedSensorPosition() * (0.1257* Math.PI) * 9.1 / 2048;
  }

  public double getRightDistance(){
    return 1.0 * r1.getSelectedSensorPosition() * kEncoderTick2Meter;
  }

  // Converting rotation from pigeon to Rotation2d
  // public Rotation2d getYaw() {
  //   double[] ypr = new double[3];
  //   // gyro.getYawPitchRoll(ypr);

  //   gyro.getGeneralStatus(generalStatus);
  //   gyro.getRawGyro(ypr);
  //   double currentAngle = gyro.getFusedHeading(fusionStatus);

  //   System.out.println(currentAngle);
  //   return (false)
  //       ? Rotation2d.fromDegrees(360 - (currentAngle))
  //       : Rotation2d.fromDegrees(currentAngle);
  // }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getFusedHeading(), 360) * (true ? -1.0 : 1.0);
  }
}
