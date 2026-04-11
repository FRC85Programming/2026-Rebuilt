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
    hoodAngleMap.put(1.45, Rotation2d.fromDegrees(71));
    hoodAngleMap.put(2.41, Rotation2d.fromDegrees(69));
    hoodAngleMap.put(2.71, Rotation2d.fromDegrees(64));
    hoodAngleMap.put(3.2, Rotation2d.fromDegrees(63));
    hoodAngleMap.put(3.6, Rotation2d.fromDegrees(60));
    hoodAngleMap.put(3.8, Rotation2d.fromDegrees(58));
    hoodAngleMap.put(4.28, Rotation2d.fromDegrees(56));
    hoodAngleMap.put(4.53, Rotation2d.fromDegrees(55));
    hoodAngleMap.put(4.63, Rotation2d.fromDegrees(54));
    hoodAngleMap.put(5.13, Rotation2d.fromDegrees(53));
  

    flywheelSpeedMap.put(1.45, 3300.0);
    flywheelSpeedMap.put(2.41, 3790.0);
    flywheelSpeedMap.put(2.71, 3800.0);
    flywheelSpeedMap.put(3.2, 3900.0);   // adjusted
    flywheelSpeedMap.put(3.6, 4000.0);   // adjusted
    flywheelSpeedMap.put(3.8, 4100.0);   // adjusted
    flywheelSpeedMap.put(4.28, 4200.0);  // adjusted
    flywheelSpeedMap.put(4.53, 4300.0);  // adjusted
    flywheelSpeedMap.put(4.63, 4400.0);  // adjusted
    flywheelSpeedMap.put(5.13, 4500.0);  // adjusted
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
