package frc.robot.utilities;

public class UnitConversion {

  // velocity, FalconFX.
  public static double nativeUnitstoRPM(double nativeUnits) {
    return (nativeUnits * 600) / 2048.0;
  }

  public static double RPMtoNativeUnits(double RPM) {
    return (RPM / 600) * 2048.0;
  }

  // position, FalconFX.
  public static double positionRotsToNativeUnits(double rots) {
    return rots * 2048;
  }

  public static double positionNativeToRots(double nativeUnits) {
    return nativeUnits / 2048;
  }

  public static double convertWPILibTrajectoryUnitsToTalonSRXNativeUnits(
      double metersPerSecond, double wheelDiameter, boolean givenMetric, int ticksPerRevolution) {
    double result = metersPerSecond;
    double circumference = 0;
    if (givenMetric) {
      circumference = Math.PI * wheelDiameter;
    } else {
      double diameterInMeters = wheelDiameter * 0.3048;
      circumference = Math.PI * diameterInMeters;
    }
    double ticksPerMeter = ticksPerRevolution / circumference;
    result = result * ticksPerMeter;
    result = result * .1;

    return result;
  }

  public static double tickSecTomSec(double falconVel){
    return falconVel * 0.127 * 9.1 * 10.0 / 2048;
  }
  

}
