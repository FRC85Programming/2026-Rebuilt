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
            double launchVelocityMps,
            double flightTimeSec
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
     * @param robotVxMetersPerSec       The robot velocity in m/s in the x direction (field relative)
     * @param robotVyMetersPerSec       The robot velocity in m/s in the y direction (field relative)
     *
     * @return Optional ShotSolution if a valid trajectory exists
     */
    public static Optional<ShotSolution> solve(
        double horizontalDistanceMeters,
        double shooterHeightMeters,
        double targetHeightMeters,
        double requiredImpactAngleRad,
        double minLaunchAngleRad,
        double maxLaunchAngleRad,
        double robotVxMetersPerSec
    ) {

        if (horizontalDistanceMeters <= 0.0) {
            return Optional.empty();
        }

        ShotSolution bestSolution = null;
        double lowestVelocity = Double.POSITIVE_INFINITY;

        for (double theta = minLaunchAngleRad;
            theta <= maxLaunchAngleRad;
            theta += ANGLE_STEP_RAD) {

            double cos = Math.cos(theta);
            double sin = Math.sin(theta);
            double tan = Math.tan(theta);

            if (Math.abs(cos) < 1e-6) continue;

            double heightTerm =
                    horizontalDistanceMeters * tan
                            + shooterHeightMeters
                            - targetHeightMeters;

            if (heightTerm <= 0.0) continue;

            double velocitySquared =
                    (GRAVITY * horizontalDistanceMeters * horizontalDistanceMeters)
                            / (2.0 * cos * cos * heightTerm);

            if (velocitySquared <= 0.0) continue;

            double velocity = Math.sqrt(velocitySquared);

            // Field-relative horizontal velocity
            double vxField = velocity * cos + robotVxMetersPerSec;
            if (vxField <= 0.0) continue;

            // Time of flight
            double time = horizontalDistanceMeters / vxField;

            // Field-relative vertical velocity at impact
            double vyFinal =
                    velocity * sin
                            - GRAVITY * time;

            if (vyFinal >= 0.0) continue;

            double impactAngle =
                    Math.atan2(Math.abs(vyFinal), vxField);

            if (impactAngle < requiredImpactAngleRad) continue;

            if (velocity < lowestVelocity) {
                lowestVelocity = velocity;
                bestSolution = new ShotSolution(theta, velocity, time);
            }
        }

        return Optional.ofNullable(bestSolution);
    }
}
