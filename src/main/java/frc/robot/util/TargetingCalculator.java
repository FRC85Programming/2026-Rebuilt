package frc.robot.util;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class TargetingCalculator {
    
    public static class TargetingSolution {
        public final double flywheelRPM;
        public final double hoodAngleDegrees;
        public final double turretAngleDegrees;
        public final List<Translation3d> fieldTrajectory3d;
        
        public TargetingSolution(double flywheelRPM, double hoodAngleDegrees, double turretAngleDegrees, List<Translation3d> fieldTrajectory3d) {
            this.flywheelRPM = flywheelRPM;
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.turretAngleDegrees = turretAngleDegrees;
            this.fieldTrajectory3d = fieldTrajectory3d;
        }
    }
    
    public static TargetingSolution calculateShot(
        Translation3d targetTranslation,
        Pose2d robotPose,
        ChassisSpeeds robotVel,
        double turretAngleRads,
        RpmConverter rpmConverter
    ) {
        Translation2d robotTranslation = robotPose.getTranslation();
        Rotation2d robotRotation = robotPose.getRotation();
        
        Translation2d toTarget2d = targetTranslation.toTranslation2d().minus(robotTranslation);
        
        double distance = toTarget2d.getNorm();
        SmartDashboard.putNumber("Distance From Hub", distance);
        
        if (distance < 1e-6) {
            return new TargetingSolution(0, 0, 0, List.of());
        }
        
        Translation2d toTargetUnit = toTarget2d.div(distance);
        
        double robotVx = 
            robotVel.vxMetersPerSecond * toTargetUnit.getX()
          + robotVel.vyMetersPerSecond * toTargetUnit.getY();
        
        double goalRPM = ShooterTable.getSetpoint(distance).flywheelRPM();
        double goalAngle = ShooterTable.getSetpoint(distance).hoodAngle().getRadians();
        
        double shotSpeed = rpmConverter.rpmToMps(goalRPM);
        
        double compensatedShotSpeed = shotSpeed - robotVx;
        compensatedShotSpeed = Math.max(compensatedShotSpeed, 0.1);
        goalRPM = rpmConverter.mpsToRpm(compensatedShotSpeed);

        double timeOfFlight = distance / compensatedShotSpeed;

        double compensatedDistance = distance - robotVx * timeOfFlight;
        goalAngle = ShooterTable.getSetpoint(compensatedDistance).hoodAngle().getRadians();
        
        double robotVy = 
            -robotVel.vxMetersPerSecond * toTargetUnit.getY()
            + robotVel.vyMetersPerSecond * toTargetUnit.getX();
        
        double lateralLeadMeters = robotVy * timeOfFlight;
        double leadAngleRad = Math.atan2(lateralLeadMeters, distance);
        
        var shooterRelativeTrajectory = 
            BallisticTrajectory3d.generate(
                rpmConverter.rpmToMps(goalRPM),
                goalAngle,
                TurretConstants.ROBOT_TO_TURRET.getZ(),
                3.0,
                0.02
            );
        
        Translation2d turretOffsetField = 
            TurretConstants.ROBOT_TO_TURRET_2D.getTranslation().rotateBy(robotRotation);
        
        Translation2d shooterFieldTranslation = 
            robotTranslation.plus(turretOffsetField);
        
        Rotation2d combinedShootingAngle = 
            robotRotation.rotateBy(new Rotation2d(turretAngleRads));
        
        List<Translation3d> fieldTrajectory3d = 
            TrajectoryTransform3d.toFieldRelative(
                shooterFieldTranslation,
                combinedShootingAngle,
                shooterRelativeTrajectory
            );
        
        Pose3d[] poses = 
            fieldTrajectory3d.stream()
                .map(p -> new Pose3d(p, new Rotation3d()))
                .toArray(Pose3d[]::new);
        
        Logger.recordOutput("Shot/Trajectory3d", poses);
        
        SmartDashboard.putNumber("Calculated Shot Angle (deg)", Math.toDegrees(goalAngle));
        SmartDashboard.putNumber("Calculated Shot Speed (mps)", rpmConverter.rpmToMps(goalRPM));
        SmartDashboard.putNumber("Calculated Shot Speed (rpm)", goalRPM);
        
        double turretAngleDegrees = targetTranslation.toTranslation2d()
            .minus(robotPose.getTranslation())
            .getAngle()
            .minus(robotRotation)
            .plus(new Rotation2d(-leadAngleRad))
            .getDegrees();
        
        return new TargetingSolution(goalRPM, Math.toDegrees(goalAngle), turretAngleDegrees, fieldTrajectory3d);
    }
    
    public interface RpmConverter {
        double rpmToMps(double rpm);
        double mpsToRpm(double mps);
    }
}
