package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.can.ctre.status_frame.StatusFrameHelper;

public class DriveTrainSubsystem extends SubsystemBase {
  private WPI_TalonFX r1 = new WPI_TalonFX(Constants.r1);
  private WPI_TalonFX r2 = new WPI_TalonFX(Constants.r2);
  private WPI_TalonFX r3 = new WPI_TalonFX(Constants.r3);
  private WPI_TalonFX l1 = new WPI_TalonFX(Constants.l1);
  private WPI_TalonFX l2 = new WPI_TalonFX(Constants.l2);
  private WPI_TalonFX l3 = new WPI_TalonFX(Constants.l3);

  public NetworkTableEntry driveLVolts;
  public NetworkTableEntry driveRVolts;

  public boolean reversed = false;

  public final DifferentialDrive kDrive;
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          Constants.ksVolts,
          Constants.kvVoltsSecondsPerMeter,
          Constants.kaVoltSecondsSquaredPerMeter);

  private PIDController left_PIDController = new PIDController(Constants.kPDriveVel, 0, 0);
  private PIDController right_PIDController = new PIDController(Constants.kPDriveVel, 0, 0);

  private final DifferentialDriveOdometry m_odometry;

  private PigeonIMU m_gryo = new PigeonIMU(Constants.pigeon);

  public DriveTrainSubsystem() {

    m_gryo.configFactoryDefault();

    kDrive = new DifferentialDrive(l1, r1);

    r1.configFactoryDefault();
    r2.configFactoryDefault();
    r3.configFactoryDefault();
    l1.configFactoryDefault();
    l2.configFactoryDefault();
    l3.configFactoryDefault();

    l1.enableVoltageCompensation(true);
    r1.enableVoltageCompensation(true);

    l2.enableVoltageCompensation(true);
    r2.enableVoltageCompensation(true);

    l3.enableVoltageCompensation(true);
    r3.enableVoltageCompensation(true);

    l1.configVoltageCompSaturation(12);
    r1.configVoltageCompSaturation(12);
    l2.configVoltageCompSaturation(12);
    r2.configVoltageCompSaturation(12);
    l3.configVoltageCompSaturation(12);
    r3.configVoltageCompSaturation(12);

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

    // this denoises our followers so we don't have to worry about the CANbus being as noisy.
    // if this causes _any_ issues comment it out and we'll be fine.
    StatusFrameHelper.setStatusFrame(StatusFrame.Status_1_General, 255, l2, l3, r2, r3);
    StatusFrameHelper.setStatusFrame(StatusFrame.Status_2_Feedback0, 255, l2, l3, r2, r3);

    l1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    r1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    resetOdometry(new Pose2d());

    zeroHeading();
  }

  @Override
  public void periodic() {
    double leftDist = getLeftPosition();
    double rightDist = getRightPosition();
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftDist, rightDist);

    Pose2d currentPose = m_odometry.getPoseMeters();

    // System.out.println("Current Pose: " + getPose().toString());
    // // DriverStation.reportError("Current Heading: " + getHeading(), false);

    // System.out.println("Right Position Traveled: " + getRightPosition());

    // System.out.println("Left Position Traveled: " + getLeftPosition());

  }

  public void setDefaultMotorConfig(){

    r1.setNeutralMode(NeutralMode.Coast);
    l1.setNeutralMode(NeutralMode.Coast);
    r2.setNeutralMode(NeutralMode.Coast);
    l2.setNeutralMode(NeutralMode.Coast);
    r3.setNeutralMode(NeutralMode.Brake);
    l3.setNeutralMode(NeutralMode.Brake);

  }

  public void setCurrentLimit(SupplyCurrentLimitConfiguration config) {
    r1.configSupplyCurrentLimit(config);
    l1.configSupplyCurrentLimit(config);
    r2.configSupplyCurrentLimit(config);
    l2.configSupplyCurrentLimit(config);
    r3.configSupplyCurrentLimit(config);
    l3.configSupplyCurrentLimit(config);

    l1.configOpenloopRamp(.2);
    l2.configOpenloopRamp(.2);
    l3.configOpenloopRamp(.2);
    r1.configOpenloopRamp(.2);
    r2.configOpenloopRamp(.2);
    r3.configOpenloopRamp(.2);
  }

  // methods to get everything in correct units

  public double getLeftPosition() {
    return l1.getSelectedSensorPosition()
        * Constants.wheelCircumeference
        * Constants.kGearReduction
        / Constants.oneRevEncodeCount;
  }

  public void setCoast() {
    r1.setNeutralMode(NeutralMode.Coast);
    l1.setNeutralMode(NeutralMode.Coast);
    r2.setNeutralMode(NeutralMode.Coast);
    l2.setNeutralMode(NeutralMode.Coast);
    r3.setNeutralMode(NeutralMode.Coast);
    l3.setNeutralMode(NeutralMode.Coast);
  }

  public void setConfig() {
    r1.setNeutralMode(NeutralMode.Coast);
    l1.setNeutralMode(NeutralMode.Coast);
    r2.setNeutralMode(NeutralMode.Coast);
    l2.setNeutralMode(NeutralMode.Coast);
    r3.setNeutralMode(NeutralMode.Brake);
    l3.setNeutralMode(NeutralMode.Brake);
  }

  public double getRightPosition() {
    return r1.getSelectedSensorPosition()
        * Constants.wheelCircumeference
        * Constants.kGearReduction
        / Constants.oneRevEncodeCount;
  }

  public double getLeftVelocity() {
    return l1.getSelectedSensorVelocity()
        * Constants.wheelCircumeference
        * Constants.kGearReduction
        * 10.0
        / Constants.oneRevEncodeCount;
  }

  public double getRightVelocity() {
    return r1.getSelectedSensorVelocity()
        * Constants.wheelCircumeference
        * Constants.kGearReduction
        * 10.0
        / Constants.oneRevEncodeCount;
  }

  // get Pose
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SimpleMotorFeedforward getFeedForward() {
    return m_feedforward;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  // this may not be needed?
  public ChassisSpeeds getChassisSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public PIDController getLeftPidController() {
    return left_PIDController;
  }

  public PIDController getRightPidController() {
    return right_PIDController;
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    if (reversed) {
      l1.setVoltage(-leftVolts);
      r1.setVoltage(-rightVolts);
    } else {
      l1.setVoltage(leftVolts);
      r1.setVoltage(rightVolts);
    }

    kDrive.feed();
  }

  public void resetEncoders() {
    l1.setSelectedSensorPosition(0);
    r1.setSelectedSensorPosition(0);
  }

  public void zeroHeading() {
    m_gryo.setFusedHeading(0.0);
    m_gryo.setAccumZAngle(0.0);
  }

  public double getHeading() {
    return Math.IEEEremainder(_getFusedHeading(), 360) * (reversed ? -1.0 : 1.0);
  }

  public void setPose(double x, double y) {
    resetOdometry(new Pose2d(x, y, Rotation2d.fromDegrees(_getFusedHeading())));
  }

  public void setPose(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(_getFusedHeading()));
  }

  public double _getFusedHeading() {
    return m_gryo.getFusedHeading();
  }
}
