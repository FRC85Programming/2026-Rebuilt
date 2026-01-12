package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation3d;

public class BallisticTrajectory3d {
    public static final double GRAVITY = 9.81;

    /**
     * Generates a 3D ballistic trajectory in shooter-relative coordinates.
     *
     * X = forward
     * Y = sideways (0 for now)
     * Z = height
     */
    public static List<Translation3d> generate(
            double launchVelocityMps,
            double launchAngleRad,
            double initialHeightMeters,
            double totalTimeSeconds,
            double timestepSeconds
    ) {
        List<Translation3d> points = new ArrayList<>();

        double vx = launchVelocityMps * Math.cos(launchAngleRad);
        double vz = launchVelocityMps * Math.sin(launchAngleRad);

        for (double t = 0.0; t <= totalTimeSeconds; t += timestepSeconds) {
            double x = vx * t;
            double z = initialHeightMeters + vz * t - 0.5 * GRAVITY * t * t;

            if (z < 0) break;

            points.add(new Translation3d(x, 0.0, z));
        }

        return points;
    }
}

