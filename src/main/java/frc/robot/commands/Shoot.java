package frc.robot.commands;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.BallisticTrajectory3d;
import frc.robot.util.ShooterTable;
import frc.robot.util.TimeOfFlightCalculator;
import frc.robot.util.TrajectoryTransform3d;


public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final Supplier<Translation3d> target;
    private double goalRPM = 0;
    private double goalAngle = 0;

    public Shoot(SwerveSubsystem swerve, ShooterSubsystem shooter, TurretSubsystem turret, Supplier<Translation3d> target) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
        this.turret = turret;

        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        goalRPM = 0;
        goalAngle = 0;
    }

    private void calculateSolution() {
        Translation3d targetTranslation = target.get();
        ChassisSpeeds robotVel = swerve.getFieldVelocity();
        Pose2d robotPose = swerve.getPose();

        // Find where the turret pivot actually is in field space (2D and 3D)
        Translation2d turretFieldPos2d =
            robotPose.getTranslation()
                .plus(TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d().rotateBy(robotPose.getRotation()));

        Translation2d toTarget = targetTranslation.toTranslation2d().minus(turretFieldPos2d);
        double distance = toTarget.getNorm();

        // Prevent using super small numbers - distance should physically never be less than half of the hub's length/width
        if (distance < 1e-6) return;

        Translation2d toTargetDir = toTarget.div(distance);

        // Radial velocity: positive = moving toward target (reduces effective distance)
        double radialVel  =  robotVel.vxMetersPerSecond * toTargetDir.getX()
                           + robotVel.vyMetersPerSecond * toTargetDir.getY();

        // Lateral velocity: positive = moving left relative to target direction
        double lateralVel = -robotVel.vxMetersPerSecond * toTargetDir.getY()
                           + robotVel.vyMetersPerSecond * toTargetDir.getX();

        // Shooter height accounts for the turret's Z offset from the robot origin
        double shooterHeightMeters = ShooterConstants.SHOOTER_HEIGHT_METERS
            + TurretConstants.ROBOT_TO_TURRET.getTranslation().getZ();

        double effectiveDistance = distance;
        double timeOfFlight = 0;

        // TODO: Three iterations should work, but it's worth trying more to see if it makes a noticeable difference, although this will cost compute time
        for (int i = 0; i < 3; i++) {
            double clampedDistance = Math.max(effectiveDistance, 0.1);
            var setpoint = ShooterTable.getSetpoint(clampedDistance);
            // TODO: Make a TOF lookup table
            timeOfFlight = TimeOfFlightCalculator.calculateTimeOfFlight(
                shooter.rpmToMps(setpoint.flywheelRPM()),
                setpoint.hoodAngle().getRadians(),
                radialVel,
                shooterHeightMeters,
                targetTranslation.getZ()
            );
            effectiveDistance = distance - (radialVel * timeOfFlight);
        }

        // Clamp so that the shooter table can't get weird values
        double clampedEffectiveDistance = Math.max(effectiveDistance, 0.1);
        var setpoint = ShooterTable.getSetpoint(clampedEffectiveDistance);

        goalRPM   = Math.abs(setpoint.flywheelRPM());
        goalAngle = setpoint.hoodAngle().getRadians();

        // --- Turret aiming with lateral lead ---
        Translation2d leadTargetFieldPos = targetTranslation.toTranslation2d()
            .plus(new Translation2d(
                lateralVel * timeOfFlight,
                toTarget.getAngle().plus(Rotation2d.fromDegrees(90))
            ));

        Translation2d turretToTarget = leadTargetFieldPos.minus(turretFieldPos2d);

        Rotation2d fieldAngle  = new Rotation2d(turretToTarget.getX(), turretToTarget.getY());
        Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());

        // Subtract mounting angle
        turret.setTurretAngle(turretAngle.getDegrees() - TurretConstants.MOUNTING_OFFSET);

        SmartDashboard.putNumber("Calced Turret Angle", turretAngle.getDegrees());

        // --- Trajectory visualization ---
        Pose3d[] poses = TrajectoryTransform3d.toFieldRelative(
            robotPose.getTranslation(),
            robotPose.getRotation(),
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

        boolean ready = shooter.flywheelAtSpeed(0.80) && shooter.hoodAtAngle(1) && turret.turretAtAngle(1);
        SmartDashboard.putBoolean("AIMED", ready);

        SmartDashboard.putBoolean("READY FLYWHEEL", shooter.flywheelAtSpeed(0.80));
        SmartDashboard.putBoolean("READY HOOD", shooter.hoodAtAngle(1));
        SmartDashboard.putBoolean("READY TURRET", turret.turretAtAngle(1));

        if (ready) {
            shooter.setFeedSpeed(0.4);
        } else {
            shooter.setFeedSpeed(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.setFeedSpeed(0);
        turret.setTurretAngle(0);
        Logger.recordOutput("Shot/Trajectory3d", new Pose3d[0]);
    }
}