package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

import org.littletonrobotics.junction.Logger;

public class AlphaMechanism3d {
  private static AlphaMechanism3d measured;

  public static AlphaMechanism3d getMeasured() {
    if (measured == null) {
      measured = new AlphaMechanism3d();
    }
    return measured;
  }

  public static Rotation2d turretAngle = Rotation2d.kZero; // Robot-relative
  public static Rotation2d hoodAngle = Rotation2d.kZero; // Relative to the ground

  /** Log the component poses and camera pose. */
  public void log(String key) {
    var turretPose =
        TurretConstants.ROBOT_TO_TURRET
            .plus(
                new Transform3d(
                    Translation3d.kZero, new Rotation3d(0.0, 0.0, turretAngle.getRadians())));
    var hoodPose =
        turretPose.plus(
            new Transform3d(
                0.105, 0.0, 0.092, new Rotation3d(0.0, -hoodAngle.getRadians(), Math.PI)));
    Logger.recordOutput(key + "/Components", turretPose, hoodPose);
  }

  public static void setTurretAngle(double angle) {
    turretAngle = new Rotation2d(angle);
  }

  public static void setHoodAngle(double angle) {
    // Angle input: 0° = forward (horizontal), 90° = straight up (vertical)
    // The Transform3d uses negative pitch, so we need to convert:
    // Input 0° (forward) → 90° internal → pitch = -90° → horizontal
    // Input 90° (up) → 0° internal → pitch = 0° → vertical
    hoodAngle = new Rotation2d(Math.toRadians(90 - angle));
  }
}