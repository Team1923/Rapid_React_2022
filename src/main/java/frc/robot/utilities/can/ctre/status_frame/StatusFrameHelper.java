// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.can.ctre.status_frame;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Add your docs here.
 *
 * <p>This class contains syntactical sugar to make setting the status frames of multiple motors
 * within the same subsystems easier.
 *
 * <p>You pass in the TalonFXs as a list and we traverse it very pythonically, but in short it just
 * turns eight lines for drive, into two. }
 */
public class StatusFrameHelper {
  public static void setStatusFrame(
      StatusFrame statusFrameToSet, int interval, WPI_TalonFX... motors) {
    for (WPI_TalonFX motor : motors) {
      motor.setStatusFramePeriod(statusFrameToSet, interval, 30);
    }
  }
}
