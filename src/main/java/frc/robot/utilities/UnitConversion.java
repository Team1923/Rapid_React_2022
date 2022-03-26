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

  public static final double convertWPILibTrajectoryUnitsToTalonSRXNativeUnits(
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

  public static final double convertTalonSRXNativeUnitsToWPILibTrajecoryUnits(
      double talonVelocity, double wheelDiameter, boolean usingMetric, int ticksPerRevolution) {
    double result = talonVelocity;
    result = result * 10; // Convert ticks/100ms to ticks/sec

    double circumference = 0;
    if (usingMetric) {
      circumference = Math.PI * wheelDiameter;
    } else {
      double diameterInMeters = wheelDiameter * 0.3048;
      circumference = Math.PI * diameterInMeters;
    }
    double metersPerTick = circumference / ticksPerRevolution;
    result = result * metersPerTick;
    return result;
  }
}
