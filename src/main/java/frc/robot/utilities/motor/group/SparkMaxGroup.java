package frc.robot.utilities.motor.group;

import frc.robot.utilities.motor.SparkMaxMotor;

public class SparkMaxGroup extends ConfigurableMotorGroup<SparkMaxMotor> {
  public SparkMaxGroup(int leaderID, int... followerIDs) {
    super(leaderID, followerIDs);
  }

  @Override
  protected SparkMaxMotor create(int deviceID) {
    return new SparkMaxMotor(deviceID, this.invert, !this.coast);
  }
}
