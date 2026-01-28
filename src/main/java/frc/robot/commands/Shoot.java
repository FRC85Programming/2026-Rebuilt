package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.BallisticTrajectory;
import frc.robot.util.BallisticTrajectory3d;
import frc.robot.util.ShotSolver;
import frc.robot.util.ShotSolver.ShotSolution;
import frc.robot.util.TrajectoryTransform3d;

public class Shoot extends Command{
    ShooterSubsystem shooter;
    SwerveSubsystem swerve;
    ShotSolution shotSolution;
    Translation3d targetTranslation;
    List<Translation3d> fieldTrajectory3d;
    double dropAngle;

    public Shoot(SwerveSubsystem swerve, ShooterSubsystem shooter, Translation3d targetTranslation, double dropAngle) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.dropAngle = dropAngle;
        this.targetTranslation = targetTranslation;
    }

    private void calculateSolution() {
        // Convert the field relative velocity into velocity relative to the goal
        ChassisSpeeds robotVel = swerve.getFieldVelocity();
        Translation2d toTarget2d =
            targetTranslation.toTranslation2d()
                .minus(swerve.getPose().getTranslation());

        double distance = toTarget2d.getNorm();

        // Protect against super short distances/pose glitches
        if (distance < 1e-6) {
            return;
        }

        Translation2d toTargetUnit = toTarget2d.div(distance);
        
        double robotVx = 
            robotVel.vxMetersPerSecond * toTargetUnit.getX() 
            + robotVel.vyMetersPerSecond * toTargetUnit.getY();

            
        var solution = ShotSolver.solve(
            swerve.getPose().getTranslation().getDistance(targetTranslation.toTranslation2d()),
            0.305,
            targetTranslation.getZ(),
            Math.toRadians(dropAngle),
            Math.toRadians(Constants.ShooterConstants.HOOD_MIN_ANGLE),
            Math.toRadians(Constants.ShooterConstants.HOOD_MAX_ANGLE),
            robotVx
        );

        if (solution.isEmpty()) {
            SmartDashboard.putString("ShotSolver Status", "NO VALID SOLUTION");
            return;
        }

        shotSolution = solution.get();

        // Create trajectories for sim visulization (could be wrapped in a sim check if the code runs slow)
        var shooterRelativeTrajectory =
            BallisticTrajectory3d.generate(
                shotSolution.launchVelocityMps(),
                shotSolution.launchAngleRad(),
                0.305, 
                3.0, 
                0.02   
            );

        fieldTrajectory3d =
            TrajectoryTransform3d.toFieldRelative(
                swerve.getPose().getTranslation(),
                swerve.getPose().getRotation(),
                shooterRelativeTrajectory
            );

        Pose3d[] poses =
            fieldTrajectory3d.stream()
                .map(p -> new Pose3d(p, new Rotation3d()))
                .toArray(Pose3d[]::new);

        Logger.recordOutput("Shot/Trajectory3d", poses);

        SmartDashboard.putNumber(
            "Calculated Shot Angle (deg)",
            Math.toDegrees(shotSolution.launchAngleRad())
        );

        SmartDashboard.putNumber(
            "Calculated Shot Speed (mps)",
            shotSolution.launchVelocityMps()
        );

        SmartDashboard.putNumber(
            "Calculated Shot Speed (rpm)",
            shooter.mpsToRPM(shotSolution.launchVelocityMps())
        );
    }


    @Override
    public void execute() {
        // Placeholder! Needs to use shot leading or use turret code instead
        swerve.aimAtPosition(targetTranslation.toTranslation2d());

        if (swerve.isAimedAtPosition(0.03)) {
            calculateSolution();
        }

        if (shotSolution != null) {
            shooter.setFlywheelRPM(
                shooter.mpsToRPM(shotSolution.launchVelocityMps())
            );
    
            shooter.setHoodAngle(
                Math.toDegrees(shotSolution.launchAngleRad())
            );
            SmartDashboard.putBoolean("Flywheel at speed", shooter.flywheelAtSpeed(200));
            SmartDashboard.putBoolean("Hood at angle", shooter.hoodAtAngle(3));


            if (shooter.flywheelAtSpeed(200) && shooter.hoodAtAngle(3) && swerve.isAimedAtPosition(0.03)) {
                 if (shooter.generateProjectileIsReady()) {
                        shooter.simulatedShot(swerve.getPose(), swerve.getFieldVelocity());
                }
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
    }
}
