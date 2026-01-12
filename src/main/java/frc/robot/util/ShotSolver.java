package frc.robot.util;

import java.util.Optional;

public final class ShotSolver {

    private static final double GRAVITY = 9.80665;

    // Increment between solutions
    private static final double ANGLE_STEP_RAD = Math.toRadians(0.25);

    private ShotSolver() {
        // Utility class
    }

    /**
     * Represents a valid ballistic shot solution.
     *
     * @param launchAngleRad  Shooter launch angle (radians)
     * @param launchVelocityMps  Exit velocity (meters per second)
     */
    public record ShotSolution(
            double launchAngleRad,
            double launchVelocityMps
    ) {}

    /**
     * Attempts to solve a ballistic shot given constraints.
     *
     * @param horizontalDistanceMeters  Horizontal distance to target
     * @param shooterHeightMeters       Height of shooter exit
     * @param targetHeightMeters        Height of target
     * @param requiredImpactAngleRad    Minimum downward impact angle (radians)
     * @param minLaunchAngleRad         Minimum allowed hood angle
     * @param maxLaunchAngleRad         Maximum allowed hood angle
     *
     * @return Optional ShotSolution if a valid trajectory exists
     */
    public static Optional<ShotSolution> solve(
            double horizontalDistanceMeters,
            double shooterHeightMeters,
            double targetHeightMeters,
            double requiredImpactAngleRad,
            double minLaunchAngleRad,
            double maxLaunchAngleRad
    ) {

        if (horizontalDistanceMeters <= 0.0) {
            return Optional.empty();
        }

        ShotSolution bestSolution = null;
        // Used as placeholder before incrementing through lower velcities
        double lowestVelocity = Double.POSITIVE_INFINITY;

        for (double theta = minLaunchAngleRad;
             theta <= maxLaunchAngleRad;
             theta += ANGLE_STEP_RAD) {

            double cos = Math.cos(theta);
            double tan = Math.tan(theta);

            // Prevent divide-by-zero / vertical shots
            if (Math.abs(cos) < 1e-6) {
                continue;
            }

            double heightTerm =
                    horizontalDistanceMeters * tan
                            + shooterHeightMeters
                            - targetHeightMeters;

            if (heightTerm <= 0.0) {
                continue;
            }

            double velocitySquared =
                    (GRAVITY * horizontalDistanceMeters * horizontalDistanceMeters)
                            / (2.0 * cos * cos * heightTerm);

            if (velocitySquared <= 0.0) {
                continue;
            }

            double velocity = Math.sqrt(velocitySquared);

            // Calculate time to reach target horizontally
            double time =
                    horizontalDistanceMeters / (velocity * cos);

            double vx = velocity * cos;
            double vy = velocity * Math.sin(theta) - GRAVITY * time;

            // Must be descending
            if (vy >= 0.0) {
                continue;
            }

            // Impact angle check
            double impactAngle =
                    Math.atan2(Math.abs(vy), vx);

            if (impactAngle < requiredImpactAngleRad) {
                continue;
            }

            // Keep the lowest-energy valid solution
            if (velocity < lowestVelocity) {
                lowestVelocity = velocity;
                bestSolution = new ShotSolution(theta, velocity);
            }
        }

        return Optional.ofNullable(bestSolution);
    }
}
