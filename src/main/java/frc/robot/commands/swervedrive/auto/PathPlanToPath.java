package frc.robot.commands.swervedrive.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;


/**
 * Drives a smooth curved path from the robot's current pose to the first waypoint
 * of the named PathPlanner path. The Bezier tangents are set so that:
 *   - the path departs directly toward the target (no outward swing), and
 *   - the path arrives at the named path's first waypoint pointing in the same
 *     direction that the named path initially travels.
 * This produces a smooth round-over rather than a sharp corner at the join point.
 * The goal rotation is set to the IdealStartingState rotation of the named path.
 *
 * The follow command is run inline (not scheduled separately) so the parent
 * PathPlanner auto group is never cancelled by a subsystem conflict.
 */
public class PathPlanToPath extends Command
{
    private final SwerveSubsystem swerve;
    private final String pathName;
    private Command followCommand;

    public PathPlanToPath(SwerveSubsystem swerve, String pathName) {
        this.swerve = swerve;
        this.pathName = pathName;
        // No addRequirements — followCommand is driven inline.
    }

    @Override
    public void initialize() {
        followCommand = null;
        try {
            PathPlannerPath targetPath = PathPlannerPath.fromPathFile(pathName);

            Pose2d currentPose    = swerve.getPose();
            Translation2d startPoint = currentPose.getTranslation();
            Waypoint firstWaypoint   = targetPath.getWaypoints().get(0);
            Translation2d endPoint   = firstWaypoint.anchor();
            Translation2d travelDelta = endPoint.minus(startPoint);

            // --- Bezier tangent headings for smooth rounding ---
            // Start pose: depart directly toward the endpoint so there is no
            // outward swing at the beginning of the approach.
            Rotation2d startHeading = new Rotation2d(travelDelta.getX(), travelDelta.getY());

            // End pose: arrive pointing in the same direction that the named path
            // initially travels.  Prefer the exact nextControl tangent stored in
            // the path file; fall back to the first→second waypoint direction.
            Rotation2d endHeading;
            Translation2d nextCtrl = firstWaypoint.nextControl();
            if (nextCtrl != null) {
                Translation2d tangent = nextCtrl.minus(endPoint);
                endHeading = new Rotation2d(tangent.getX(), tangent.getY());
            } else if (targetPath.getWaypoints().size() > 1) {
                Translation2d secondAnchor = targetPath.getWaypoints().get(1).anchor();
                Translation2d tangent = secondAnchor.minus(endPoint);
                endHeading = new Rotation2d(tangent.getX(), tangent.getY());
            } else {
                endHeading = new Rotation2d(travelDelta.getX(), travelDelta.getY());
            }

            Pose2d[] poses = new Pose2d[]{
                new Pose2d(startPoint, startHeading),
                new Pose2d(endPoint,   endHeading)
            };

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

            // Holonomic rotation target at the start so the robot begins turning early
            // toward the goal body rotation.
            List<RotationTarget> rotationTargets = new ArrayList<>();
            rotationTargets.add(new RotationTarget(0, new Rotation2d(travelDelta.getX(), travelDelta.getY())));

            // Goal rotation = the starting rotation of the named path.
            // Fall back to the travel direction if the path has no IdealStartingState.
            Rotation2d goalRotation;
            IdealStartingState startState = targetPath.getIdealStartingState();
            if (startState != null) {
                goalRotation = startState.rotation();
            } else {
                goalRotation = new Rotation2d(travelDelta.getX(), travelDelta.getY());
            }

            PathConstraints constraints = new PathConstraints(
                swerve.getSwerveDrive().getMaximumChassisVelocity()*0.7, 3.0,
                swerve.getSwerveDrive().getMaximumChassisAngularVelocity(),
                Units.degreesToRadians(720));

            PathPlannerPath ppPath = new PathPlannerPath(
                waypoints, rotationTargets, List.of(), List.of(), List.of(),
                constraints, null,
                new GoalEndState(0.0, goalRotation), false);
            ppPath.preventFlipping = true;

            followCommand = AutoBuilder.followPath(ppPath);
            followCommand.initialize();
        } catch (Exception e) {
            System.err.println("[PathPlanToPath] Could not load path '" + pathName + "': " + e.getMessage());
        }
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
}
