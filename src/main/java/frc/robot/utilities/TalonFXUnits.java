// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class TalonFXUnits {

  public static double rpmToNativeUnits(double rpm) {

    /*
     * Convert 500 RPM to units / 100ms.
     * 2048 Units/Rev * 500 RPM / 600 100ms/min in either direction:
     * velocity setpoint is in units/100ms
     */

    return (rpm / 600) * 2048;
  }

  public static double NativeUnitsToRpm(double nativeTicks) {
    return (nativeTicks * 600) / 2048;
  }
}
