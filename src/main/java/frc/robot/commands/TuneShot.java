package frc.robot.commands;

import java.util.List;
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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.BallisticTrajectory;
import frc.robot.util.BallisticTrajectory3d;
import frc.robot.util.ShooterTable;
import frc.robot.util.ShotSolver;
import frc.robot.util.ShotSolver.ShotSolution;
import frc.robot.util.TrajectoryTransform3d;

public class TuneShot extends Command{
    ShooterSubsystem shooter;
    SwerveSubsystem swerve;
    Supplier<Translation3d> target;
    double goalRPM = 0;
    double goalAngle = 0;
    Translation3d targetTranslation;
    TurretSubsystem turret;
    IndexerSubsystem indexer;

    public TuneShot(SwerveSubsystem swerve, ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret, Supplier<Translation3d> target) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
        this.turret = turret;

        SmartDashboard.putBoolean("Feed", false);
        SmartDashboard.putNumber("Feed Speed", 0.8);

        SmartDashboard.putNumber("TUNE RPM", 1000);
        SmartDashboard.putNumber("TUNE ANGLE", 75);
    }

    private void calculateSolution() {
        targetTranslation = target.get();

        // Distance from SHOOTER to hub
        Pose2d robotPose = swerve.getPose();

        Translation2d shooterFieldPosition =
            robotPose.getTranslation().plus(
                ShooterConstants.SHOOTER_TRANSLATION.rotateBy(robotPose.getRotation())
            );

        Translation2d toTarget2d =
            targetTranslation.toTranslation2d().minus(shooterFieldPosition);


        double distance = toTarget2d.getNorm();

        SmartDashboard.putNumber("TUNE Distance To Hub", distance);

        // Protect against super short distances/pose glitches
        if (distance < 1e-6) {
            return;
        }

            
        /**goalRPM = SmartDashboard.getNumber("TUNE Shot RPM", 0);
        goalAngle = SmartDashboard.getNumber("TUNE Shot Angle", 80);*/

        goalRPM = SmartDashboard.getNumber("TUNE RPM", 1000);
        goalAngle = SmartDashboard.getNumber("TUNE ANGLE", 75);

        // Find where the turret pivot actually is in field space
        Translation2d turretFieldPos =
            robotPose.getTranslation()
                .plus(TurretConstants.ROBOT_TO_TURRET_2D.getTranslation().rotateBy(robotPose.getRotation()));

        Translation2d turretToTarget = targetTranslation.toTranslation2d().minus(turretFieldPos);
        Rotation2d fieldAngle = new Rotation2d(turretToTarget.getX(), turretToTarget.getY());
        Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());

        // Subtract mounting angle
        turret.setTurretAngle(turretAngle.getDegrees() - TurretConstants.MOUNTING_OFFSET);

        SmartDashboard.putNumber("Selected Angle", goalAngle);
        SmartDashboard.putNumber("Selected RPM", goalRPM);

        //swerve.aimAtPositionWithLead(targetTranslation.toTranslation2d(), 0, false);
    }

    @Override
    public void execute() {
        calculateSolution();

        shooter.setFlywheelRPM(
            goalRPM
        );
    
        shooter.setHoodAngle(
            // In degrees
            goalAngle
        );

        if (SmartDashboard.getBoolean("Feed", false)) {
            indexer.startIndexing();
        }

        SmartDashboard.putNumber("Ball Exit Speed", shooter.rpmToMps(goalRPM));
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
        indexer.stopIndexing();
        shooter.stopFlywheel();
        turret.setTurretAngle(0);
    }
}
