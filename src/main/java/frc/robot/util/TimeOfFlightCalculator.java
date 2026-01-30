package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TimeOfFlightCalculator {
    private static final double GRAVITY = 9.80665; 
    private static final int MAX_ITERS = 15;

    /**
     * Computes time of flight for a shot using iterative projectile math.
     *
     * @param shooterPos Field-relative shooter position
     * @param targetPos Field-relative target position
     * @param robotVelocity Field-relative robot velocity
     * @param exitSpeedMetersPerSec Ball exit speed
     * @param shooterAngleRadians Hood angle
     */
    public static double solveTimeOfFlight(
        Translation2d shooterPos,
        Translation2d targetPos,
        ChassisSpeeds robotVelocity,
        double exitSpeedMetersPerSec,
        double shooterAngleRadians) {

        Translation2d delta = targetPos.minus(shooterPos);
        double horizontalDistance = delta.getNorm();

        // Initial guess (flat shot)
        double time = horizontalDistance / exitSpeedMetersPerSec;

        for (int i = 0; i < MAX_ITERS; i++) {
        double vx =
            exitSpeedMetersPerSec * Math.cos(shooterAngleRadians)
                + robotVelocity.vxMetersPerSecond;

        double vy =
            exitSpeedMetersPerSec * Math.sin(shooterAngleRadians)
                + robotVelocity.vyMetersPerSecond;

        double x = vx * time;
        double y = vy * time - 0.5 * GRAVITY * time * time;

        double error = horizontalDistance - Math.hypot(x, y);

        // Simple Newton-style correction
        time += error / Math.max(exitSpeedMetersPerSec, 0.1);
        }

        return Math.max(time, 0.0);
    }
}
