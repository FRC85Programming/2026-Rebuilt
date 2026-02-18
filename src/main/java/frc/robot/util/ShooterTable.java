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
    hoodAngleMap.put(1.468, Rotation2d.fromDegrees(78));
    hoodAngleMap.put(2.109, Rotation2d.fromDegrees(75));
    hoodAngleMap.put(2.436, Rotation2d.fromDegrees(73));
    hoodAngleMap.put(3.017, Rotation2d.fromDegrees(70));
    hoodAngleMap.put(3.396, Rotation2d.fromDegrees(68));
    hoodAngleMap.put(3.765, Rotation2d.fromDegrees(67));
    hoodAngleMap.put(4.003, Rotation2d.fromDegrees(66));
    hoodAngleMap.put(4.34, Rotation2d.fromDegrees(65));
    hoodAngleMap.put(4.784, Rotation2d.fromDegrees(65));
    hoodAngleMap.put(5.166, Rotation2d.fromDegrees(64));

    flywheelSpeedMap.put(1.468, 3500.0); 
    flywheelSpeedMap.put(2.109, 3600.0); 
    flywheelSpeedMap.put(2.436, 3700.0); 
    flywheelSpeedMap.put(3.017, 3800.0); 
    flywheelSpeedMap.put(3.396, 3733.0); 
    flywheelSpeedMap.put(3.765, 4100.0); 
    flywheelSpeedMap.put(4.003, 4200.0); 
    flywheelSpeedMap.put(4.34, 4250.0); 
    flywheelSpeedMap.put(4.784, 4500.0); 
    flywheelSpeedMap.put(5.166, 4800.0); 







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
