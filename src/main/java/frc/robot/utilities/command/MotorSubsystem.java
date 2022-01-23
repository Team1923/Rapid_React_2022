package frc.robot.utilities.command;

import frc.robot.utilities.motor.Motor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MotorSubsystem extends Subsystem {
  private List<Motor> motors;

  public MotorSubsystem(Motor... motors) {
    this.motors = new ArrayList<>(Arrays.asList(motors));
  }

  protected void add(Motor... motors) {
    this.motors.addAll(Arrays.asList(motors));
  }

  @Override
  public void stop() {
    for (Motor motor : this.motors) motor.stop();
  }
}
