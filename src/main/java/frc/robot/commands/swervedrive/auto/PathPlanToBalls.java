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

    public PathPlanToBalls(SwerveSubsystem swerve, VisionSubsystem vision, double direction) {
        this.swerve = swerve;
        this.vision = vision;
        finder = null;
        this.direction = direction;
    }

    @Override
    public void initialize() {
        phase = Phase.CALCULATING;
        path = null;
        followCommand = null;
        finder = new BallPathCalculator(balls, swerve.getPose().getTranslation(), direction);
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
