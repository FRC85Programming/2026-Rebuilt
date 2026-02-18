package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class TimeOfFlightCalculator {

    private static final double GRAVITY = 9.81;

    /**
     * Computes time of flight considering robot movement and goal height.
     * * @param exitSpeedMetersPerSec The speed of the ball leaving the shooter.
     * @param shooterAngleRad The angle of the shooter (0 is horizontal).
     * @param robotVelocityMps The velocity of the robot in the direction of the shot.
     * @param shooterHeight The height of the shooter exit from the ground.
     * @param targetHeight The height of the center of the target.
     * @return The time of flight in seconds, or NaN if the shot is impossible.
     */
    public static double calculateTimeOfFlight(
        double exitSpeedMetersPerSec, 
        double shooterAngleRad,
        double robotVelocityMps,
        double shooterHeight, 
        double targetHeight
    ) {
        // 1. Initial vertical velocity (y-component)
        double vy = exitSpeedMetersPerSec * Math.sin(shooterAngleRad);
        
        // Note: Horizontal velocity (vx) doesn't affect TIME in a vacuum, 
        // but it's affected by robotVelocityMps for distance calculations later!

        // 2. Setup Quadratic: 0 = -0.5*g*t^2 + vy*t + (h0 - h_target)
        // Standard form: at^2 + bt + c = 0
        double a = -0.5 * GRAVITY;
        double b = vy;
        double c = shooterHeight - targetHeight;

        double discriminant = (b * b) - (4 * a * c);

        // If discriminant is negative, the ball never reaches the target height :(
        if (discriminant < 0) {
            return Double.NaN;
        }

        double sqrtDisc = Math.sqrt(discriminant);
        double t1 = (-b + sqrtDisc) / (2 * a);
        double t2 = (-b - sqrtDisc) / (2 * a);

        // We want the largest positive time (the downward arc/lob shot)
        double time = Math.max(t1, t2);

        return (time > 0) ? time : Double.NaN;
    }
}