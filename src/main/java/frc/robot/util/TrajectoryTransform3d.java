package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public final class TrajectoryTransform3d {

    private TrajectoryTransform3d() {} // prevent instantiation

    public static List<Translation3d> toFieldRelative(
            Translation2d shooterPosition,
            Rotation2d shooterYaw,
            List<Translation3d> trajectory
    ) {
        List<Translation3d> fieldPoints = new ArrayList<>();

        for (Translation3d p : trajectory) {
            Translation2d rotatedXY =
                new Translation2d(p.getX(), p.getY())
                    .rotateBy(shooterYaw);

            fieldPoints.add(
                new Translation3d(
                    shooterPosition.getX() + rotatedXY.getX(),
                    shooterPosition.getY() + rotatedXY.getY(),
                    p.getZ()
                )
            );
        }

        return fieldPoints;
    }
}
