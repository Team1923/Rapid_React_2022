package frc.robot.utilities;

public class UnitConversion {

  public static double nativeUnitstoRPM(double nativeUnits) {
    return (nativeUnits * 600) / 2048.0;
  }

  public static double RPMtoNativeUnits(double RPM) {
    return (RPM / 600) * 2048.0;
  }
}
