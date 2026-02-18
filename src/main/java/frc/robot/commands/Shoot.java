package frc.robot.commands;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.BallisticTrajectory3d;
import frc.robot.util.ShooterTable;
import frc.robot.util.TimeOfFlightCalculator;
import frc.robot.util.TrajectoryTransform3d;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final Supplier<Translation3d> target;
    private final boolean isPathPlanner;
    private double goalRPM = 0;
    private double goalAngle = 0;

    public Shoot(SwerveSubsystem swerve, ShooterSubsystem shooter, Supplier<Translation3d> target, boolean isPathPlanner) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
        this.isPathPlanner = isPathPlanner;
    }

    @Override
    public void initialize() {
        shooter.cleanupPieces();
    }

    private void calculateSolution() {
        Translation3d targetTranslation = target.get();
        ChassisSpeeds robotVel = swerve.getFieldVelocity();
        Translation2d toTarget = targetTranslation.toTranslation2d().minus(swerve.getPose().getTranslation());
        double distance = toTarget.getNorm();

        if (distance < 1e-6) return;

        Translation2d toTargetDir = toTarget.div(distance);
        double radialVel  =  robotVel.vxMetersPerSecond * toTargetDir.getX() + robotVel.vyMetersPerSecond * toTargetDir.getY();
        double lateralVel = -robotVel.vxMetersPerSecond * toTargetDir.getY() + robotVel.vyMetersPerSecond * toTargetDir.getX();

        double effectiveDistance = distance;
        double timeOfFlight = 0;

        for (int i = 0; i < 3; i++) {
            var setpoint = ShooterTable.getSetpoint(effectiveDistance);
            timeOfFlight = TimeOfFlightCalculator.calculateTimeOfFlight(
                shooter.rpmToMps(setpoint.flywheelRPM()),
                setpoint.hoodAngle().getRadians(),
                radialVel,
                ShooterConstants.SHOOTER_HEIGHT_METERS,
                targetTranslation.getZ()
            );
            effectiveDistance = distance - (radialVel * timeOfFlight);
        }

        var setpoint = ShooterTable.getSetpoint(effectiveDistance);
        goalRPM   = setpoint.flywheelRPM();
        goalAngle = setpoint.hoodAngle().getRadians();

        swerve.aimAtPositionWithLead(
            targetTranslation.toTranslation2d(),
            Math.atan2(lateralVel * timeOfFlight, distance),
            isPathPlanner
        );

        Pose3d[] poses = TrajectoryTransform3d.toFieldRelative(
            swerve.getPose().getTranslation(),
            swerve.getPose().getRotation(),
            BallisticTrajectory3d.generate(shooter.rpmToMps(goalRPM), goalAngle, 0.305, 3.0, 0.02)
        ).stream()
            .map(p -> new Pose3d(p, new Rotation3d()))
            .toArray(Pose3d[]::new);

        Logger.recordOutput("Shot/Trajectory3d", poses);
        SmartDashboard.putNumber("Calculated Shot Angle (deg)", Math.toDegrees(goalAngle));
        SmartDashboard.putNumber("Calculated Shot Speed (rpm)", goalRPM);
    }

    @Override
    public void execute() {
        calculateSolution();
        shooter.setFlywheelRPM(goalRPM);
        shooter.setHoodAngle(Math.toDegrees(goalAngle));

        boolean ready = shooter.flywheelAtSpeed(0.95) && shooter.hoodAtAngle(0.5) && swerve.isAimedAtPosition(0.1);
        SmartDashboard.putBoolean("AIMED", ready);

        if (ready) {
            shooter.setFeedSpeed(0.8);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
        shooter.stopFlywheel();
        shooter.setFeedSpeed(0);
        Logger.recordOutput("Shot/Trajectory3d", new Pose3d[0]);
        swerve.resetPathPlannerRotOverride();
    }
}
