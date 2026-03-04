package frc.robot.commands.swervedrive.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.BallPathCalculator;


public class PathPlanToBalls extends Command
{
    private enum Phase { CALCULATING, FOLLOWING }

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private BallPathCalculator finder;
    private Translation2d[] path;
    private Command followCommand;
    private Phase phase;
    private Translation2d[] balls;
    private double direction;

    /**
     * Y sweep limits, each 1 m past the field centre line (4.04 m).
     *   direction = +1 → upper cap  (don't sweep above SWEEP_LIMIT_NORTH going north)
     *   direction = -1 → lower cap  (don't sweep below SWEEP_LIMIT_SOUTH going south)
     */
    private static final double SWEEP_LIMIT_NORTH = 5.04; // field centre + 1 m
    private static final double SWEEP_LIMIT_SOUTH = 3.04; // field centre - 1 m

    /** The pre-baked ball list to use instead of querying vision (may be null). */
    private final Translation2d[] preloadedBalls;

    public PathPlanToBalls(SwerveSubsystem swerve, VisionSubsystem vision, double direction) {
        this(swerve, vision, direction, null);
    }

    /**
     * @param preloadedBalls If non-null, these balls are used instead of calling vision.
     *                       Pass {@code RobotContainer.getTestBalls()} in simulation.
     */
    public PathPlanToBalls(SwerveSubsystem swerve, VisionSubsystem vision,
                           double direction, Translation2d[] preloadedBalls) {
        this.swerve = swerve;
        this.vision = vision;
        finder = null;
        this.direction = direction;
        this.preloadedBalls = preloadedBalls;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        phase = Phase.CALCULATING;
        balls = vision.getAllBallFieldTranslations()
                          .stream()
                          .map(t -> new Translation2d(t.getX(), t.getY()))
                          .toArray(Translation2d[]::new);
        path = null;
        followCommand = null;
        double sweepYLimit = direction > 0 ? SWEEP_LIMIT_NORTH : SWEEP_LIMIT_SOUTH;
        finder = new BallPathCalculator(balls, swerve.getPose().getTranslation(), direction, sweepYLimit);
    }

    @Override
    public void execute() {

        if (phase == Phase.CALCULATING) {
            path = finder.getBestPath();
            Logger.recordOutput("FieldSimulation/calcualtedPath", path);
            if (path != null && path.length > 1) {
                followCommand = swerve.driveTranslation2dPath(path);
                followCommand.initialize();
                phase = Phase.FOLLOWING;
            }
        } else if (phase == Phase.FOLLOWING) {
            followCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (phase == Phase.CALCULATING) {
            // End early only if path computation produced no usable path.
            return path != null && path.length <= 1;
        }
        return followCommand != null && followCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (followCommand != null) {
            followCommand.end(interrupted);
        }
    }
}