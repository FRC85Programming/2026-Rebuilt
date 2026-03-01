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
    hoodAngleMap.put(1.89, Rotation2d.fromDegrees(71));
    hoodAngleMap.put(2.29, Rotation2d.fromDegrees(69));
    hoodAngleMap.put(2.82, Rotation2d.fromDegrees(67));
    hoodAngleMap.put(3.32, Rotation2d.fromDegrees(65));
    hoodAngleMap.put(3.85, Rotation2d.fromDegrees(62));
    hoodAngleMap.put(4.05, Rotation2d.fromDegrees(60));
    hoodAngleMap.put(4.89, Rotation2d.fromDegrees(56));
  

    flywheelSpeedMap.put(1.89, 3160.0); 
    flywheelSpeedMap.put(2.29, 3356.0); 
    flywheelSpeedMap.put(2.82, 3355.0); 
    flywheelSpeedMap.put(3.32, 3452.0); 
    flywheelSpeedMap.put(3.85, 3550.0); 
    flywheelSpeedMap.put(4.05, 3655.0); 
    flywheelSpeedMap.put(4.89, 3852.0); 
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
