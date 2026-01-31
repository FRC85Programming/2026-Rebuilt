package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.BallisticTrajectory3d;
import frc.robot.util.ShooterTable;
import frc.robot.util.TrajectoryTransform3d;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final Supplier<Translation3d> target;

    private List<Translation3d> fieldTrajectory3d;

    private double goalRPM = 0.0;
    private double goalAngle = 0.0;

    private final boolean isPathPlanner;
    private Translation3d targetTranslation;

    public Shoot(
        SwerveSubsystem swerve,
        ShooterSubsystem shooter,
        Supplier<Translation3d> target,
        boolean isPathPlanner
    ) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
        this.isPathPlanner = isPathPlanner;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.cleanupPieces();
    }

    private void calculateSolution() {
        targetTranslation = target.get();

        // Robot pose
        var robotPose = swerve.getPose();
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();

        // Vector from robot center to target
        Translation2d toTarget2d =
            targetTranslation.toTranslation2d().minus(robotTranslation);

        double distance = toTarget2d.getNorm();
        SmartDashboard.putNumber("Distance From Hub", distance);

        if (distance < 1e-6) {
            return;
        }

        Translation2d toTargetUnit = toTarget2d.div(distance);

        // Robot velocity along shot direction
        ChassisSpeeds robotVel = swerve.getFieldVelocity();
        double robotVx =
            robotVel.vxMetersPerSecond * toTargetUnit.getX()
          + robotVel.vyMetersPerSecond * toTargetUnit.getY();

        // Base lookup
        goalRPM = ShooterTable.getSetpoint(distance).flywheelRPM();
        goalAngle = ShooterTable.getSetpoint(distance).hoodAngle().getRadians();

        // Velocity compensation (optional but you already had this)
        double shotSpeed = shooter.rpmToMps(goalRPM);
        double timeOfFlight = distance / shotSpeed;

        double compensatedShotSpeed = shotSpeed - robotVx;
        goalRPM = shooter.mpsToRpm(compensatedShotSpeed);

        double compensatedDistance = distance - robotVx * timeOfFlight;
        goalAngle =
            ShooterTable.getSetpoint(compensatedDistance)
                .hoodAngle()
                .getRadians();

        // Lateral lead
        double robotVy =
            -robotVel.vxMetersPerSecond * toTargetUnit.getY()
            + robotVel.vyMetersPerSecond * toTargetUnit.getX();

        double lead = robotVy * timeOfFlight;

        swerve.aimAtPositionWithLead(
            targetTranslation.toTranslation2d(),
            -lead,
            isPathPlanner
        );


        var shooterRelativeTrajectory =
            BallisticTrajectory3d.generate(
                shooter.rpmToMps(goalRPM),
                goalAngle,
                Constants.ShooterConstants.SHOOTER_HEIGHT_METERS,
                3.0,
                0.02
            );

        // Rotate turret offset into field frame
        Translation2d turretOffsetField =
            TurretConstants.ROBOT_TO_TURRET_2D.getTranslation().rotateBy(robotRotation);

        // Shooter origin in field space
        Translation2d shooterFieldTranslation =
            robotTranslation.plus(turretOffsetField);

        // Convert to field-relative trajectory
        fieldTrajectory3d =
            TrajectoryTransform3d.toFieldRelative(
                shooterFieldTranslation,
                robotRotation,
                shooterRelativeTrajectory
            );

        Pose3d[] poses =
            fieldTrajectory3d.stream()
                .map(p -> new Pose3d(p, new Rotation3d()))
                .toArray(Pose3d[]::new);

        Logger.recordOutput("Shot/Trajectory3d", poses);

        SmartDashboard.putNumber(
            "Calculated Shot Angle (deg)",
            Math.toDegrees(goalAngle)
        );

        SmartDashboard.putNumber(
            "Calculated Shot Speed (mps)",
            shooter.rpmToMps(goalRPM)
        );

        SmartDashboard.putNumber(
            "Calculated Shot Speed (rpm)",
            goalRPM
        );
    }

    @Override
    public void execute() {
        calculateSolution();

        shooter.setFlywheelRPM(goalRPM);
        shooter.setHoodAngle(Math.toDegrees(goalAngle));

        SmartDashboard.putBoolean(
            "Flywheel at speed",
            shooter.flywheelAtSpeed(200)
        );

        SmartDashboard.putBoolean(
            "Hood at angle",
            shooter.hoodAtAngle(3)
        );

        if (
            shooter.flywheelAtSpeed(200)
            && shooter.hoodAtAngle(3)
            && swerve.isAimedAtPosition(0.1)
        ) {
            if (shooter.generateProjectileIsReady()) {
                shooter.simulatedShot(
                    swerve.getPose(),
                    swerve.getFieldVelocity()
                );
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
        Logger.recordOutput("Shot/Trajectory3d", new Pose3d[0]);
        swerve.resetPathPlannerRotOverride();
    }
}
