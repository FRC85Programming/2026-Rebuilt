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
    hoodAngleMap.put(1.45, Rotation2d.fromDegrees(73));
    hoodAngleMap.put(2.41, Rotation2d.fromDegrees(70));
    hoodAngleMap.put(2.71, Rotation2d.fromDegrees(67));
    hoodAngleMap.put(3.2, Rotation2d.fromDegrees(65));
    hoodAngleMap.put(3.6, Rotation2d.fromDegrees(63));
    hoodAngleMap.put(3.8, Rotation2d.fromDegrees(61));
    hoodAngleMap.put(4.28, Rotation2d.fromDegrees(58));
    hoodAngleMap.put(4.53, Rotation2d.fromDegrees(57));
    hoodAngleMap.put(4.63, Rotation2d.fromDegrees(56));
    hoodAngleMap.put(5.13, Rotation2d.fromDegrees(54));
    hoodAngleMap.put(6.07, Rotation2d.fromDegrees(52));
  

    flywheelSpeedMap.put(1.45, 3525.0); 
    flywheelSpeedMap.put(2.41, 3775.0); 
    flywheelSpeedMap.put(2.71, 3975.0); 
    flywheelSpeedMap.put(3.2, 4175.0); 
    flywheelSpeedMap.put(3.6, 4300.0); 
    flywheelSpeedMap.put(3.8, 4425.0); 
    flywheelSpeedMap.put(4.28, 4425.0); 
    flywheelSpeedMap.put(4.53, 4475.0); 
    flywheelSpeedMap.put(4.63, 4625.0); 
    flywheelSpeedMap.put(5.13, 4725.0); 
    flywheelSpeedMap.put(6.07, 5125.0); 
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
