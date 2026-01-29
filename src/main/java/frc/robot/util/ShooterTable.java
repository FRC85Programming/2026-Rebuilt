package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterTable {
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  static {
    // This is data is calculated based off of basic physics math and should be tuned (meters, radians, RPM)
    // These were calculated with a drop angle of 65 degrees
    hoodAngleMap.put(2.0, Rotation2d.fromDegrees(75.0));
    hoodAngleMap.put(3.0, Rotation2d.fromDegrees(72.5));
    hoodAngleMap.put(4.0, Rotation2d.fromDegrees(71.25));
    hoodAngleMap.put(5.0, Rotation2d.fromDegrees(70.25));
    hoodAngleMap.put(6.0, Rotation2d.fromDegrees(69.5));



    flywheelSpeedMap.put(2.0, 2633.0); 
    flywheelSpeedMap.put(3.0, 2931.0); 
    flywheelSpeedMap.put(4.0, 3226.0); 
    flywheelSpeedMap.put(5.0, 3489.0); 
    flywheelSpeedMap.put(6.0, 3733.0); 


  }

  public static ShooterSetpoint getSetpoint(double distanceMeters) {
    return new ShooterSetpoint(
        hoodAngleMap.get(distanceMeters),
        flywheelSpeedMap.get(distanceMeters));
  }

  public record ShooterSetpoint(
      Rotation2d hoodAngle,
      double flywheelRPM) {}
}
