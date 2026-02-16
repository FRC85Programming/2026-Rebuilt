package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TimeOfFlightCalculator {

    /**
     * Computes time of flight for a shot using iterative projectile math.
     *
     * @param shooterPos Field-relative shooter position
     * @param targetPos Field-relative target position
     * @param robotVelocity Field-relative robot velocity
     * @param exitSpeedMetersPerSec Ball exit speed
     * @param shooterAngleRadians Hood angle
     */
    public static double calculateTimeOfFlight(
        double velocity,
        double angle,
        double shooterHeight,
        double targetHeight
    ) {
        double g = 9.81;

        double a = 0.5 * g;
        double b = -velocity * Math.sin(angle);
        double c = targetHeight - shooterHeight;

        double discriminant = b*b - 4*a*c;

        if (discriminant < 0) {
            return 0.0; // no solution
        }

        double t1 = (-b + Math.sqrt(discriminant)) / (2*a);
        double t2 = (-b - Math.sqrt(discriminant)) / (2*a);

        return Math.max(t1, t2); // use positive root
    }

}
