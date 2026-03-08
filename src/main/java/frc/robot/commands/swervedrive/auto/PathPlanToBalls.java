package frc.robot.commands.swervedrive.auto;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.BallPathCalculator;


public class PathPlanToBalls extends Command
{
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final Translation2d[] preloadedBalls;
    private final double xMin, xMax, yMin, yMax;

    private Command followCommand;

    public PathPlanToBalls(SwerveSubsystem swerve, VisionSubsystem vision) {
        this(swerve, vision, null,
             Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
             Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public PathPlanToBalls(SwerveSubsystem swerve, VisionSubsystem vision,
                           Translation2d[] preloadedBalls) {
        this(swerve, vision, preloadedBalls,
             Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
             Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public PathPlanToBalls(SwerveSubsystem swerve, VisionSubsystem vision,
                           double xMin, double xMax, double yMin, double yMax) {
        this(swerve, vision, null, xMin, xMax, yMin, yMax);
    }


    public PathPlanToBalls(SwerveSubsystem swerve, VisionSubsystem vision,
                           Translation2d[] preloadedBalls,
                           double xMin, double xMax, double yMin, double yMax) {
        this.swerve = swerve;
        this.vision = vision;
        this.preloadedBalls = preloadedBalls;
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        followCommand = null;

        if (vision.getAllBallFieldTranslations().isEmpty()) return;

        Translation2d[] rawBalls = fetchBalls();

        Translation2d robotPos = swerve.getPose().getTranslation();

        Translation2d[] balls = Arrays.stream(rawBalls)
            .filter(b -> b.getX() >= xMin && b.getX() <= xMax)
            .filter(b -> b.getY() >= yMin && b.getY() <= yMax)
            .toArray(Translation2d[]::new);

        if (balls.length == 0) return;

        double direction = calcDirection(robotPos, balls);

        Translation2d[] path = new BallPathCalculator(balls, robotPos, direction).getBestPath();
        Logger.recordOutput("FieldSimulation/calculatedPath", path);

        if (path.length >= 1 && pathLength(robotPos, path) >= 0.4) {
            followCommand = swerve.driveTranslation2dPath(path, robotPos);
            followCommand.initialize();
        }
    }

    /** Total arc length of the full path including the robot-start-to-first-waypoint segment. */
    private static double pathLength(Translation2d start, Translation2d[] path) {
        double length = start.getDistance(path[0]);
        for (int i = 0; i < path.length - 1; i++) {
            length += path[i].getDistance(path[i + 1]);
        }
        return length;
    }

    @Override
    public void execute() {
        if (followCommand != null) {
            followCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return followCommand == null || followCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (followCommand != null) {
            followCommand.end(interrupted);
        }
    }

    private Translation2d[] fetchBalls() {
        if (preloadedBalls != null) return preloadedBalls;
        return vision.getAllBallFieldTranslations()
                     .stream()
                     .map(Translation3d::toTranslation2d)
                     .toArray(Translation2d[]::new);
    }

    private static double calcDirection(Translation2d robot, Translation2d[] balls) {
        long north = Arrays.stream(balls).filter(b -> b.getY() > robot.getY()).count();
        long south = balls.length - north;
        return north >= south ? 1.0 : -1.0;
    }
}
