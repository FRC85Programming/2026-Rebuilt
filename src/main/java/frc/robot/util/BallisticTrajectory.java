package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BallisticTrajectory {
    public static final double GRAVITY = 9.81;

    public static List<Translation2d> generateTrajectory(
            double launchVelocityMps,
            double launchAngleRad,
            double initialHeightMeters,
            double totalTimeSeconds,
            double timestepSeconds
    ) {
        List<Translation2d> points = new ArrayList<>();

        double vx = launchVelocityMps * Math.cos(launchAngleRad);
        double vy = launchVelocityMps * Math.sin(launchAngleRad);

        for (double t = 0.0; t <= totalTimeSeconds; t += timestepSeconds) {
            double x = vx * t;
            double y = initialHeightMeters + vy * t - 0.5 * GRAVITY * t * t;

            if (y < 0) break; // hit ground

            points.add(new Translation2d(x, y));
        }

        return points;
    }

    public static List<Translation2d> toFieldRelative(
            Translation2d shooterPosition,
            Rotation2d shooterHeading,
            List<Translation2d> trajectory
    ) {
        List<Translation2d> fieldPoints = new ArrayList<>();

        for (Translation2d p : trajectory) {
            fieldPoints.add(
                shooterPosition.plus(p.rotateBy(shooterHeading))
            );
        }

        return fieldPoints;
    }
}

