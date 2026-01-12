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
    Translation2d targetTranslation = new Translation2d(4.620, 4.030); // Rebuilt BLUE goal
    List<Translation3d> fieldTrajectory3d;
    boolean calculated = false;
    double targetHeight = 1.829; // Rebuilt goal height


    public Shoot(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.swerve = swerve;
    }

    private void calculateSolution() {
        var solution = ShotSolver.solve(
            swerve.getPose().getTranslation().getDistance(targetTranslation),
            0.305,
            targetHeight,
            Math.toRadians(65),
            Math.toRadians(Constants.ShooterConstants.HOOD_MIN_ANGLE),
            Math.toRadians(Constants.ShooterConstants.HOOD_MAX_ANGLE)
        );

        if (solution.isEmpty()) {
            SmartDashboard.putString("ShotSolver Status", "NO VALID SOLUTION");
            return;
        }

        shotSolution = solution.get();

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
        swerve.aimAtPosition(targetTranslation);

        if (swerve.isAimedAtPosition(0.03) && !calculated) {
            calculateSolution();
            calculated = true;
        }
        if (shotSolution != null) {
            shooter.setFlywheelRPM(
                shooter.mpsToRPM(shotSolution.launchVelocityMps())
            );
    
            shooter.setHoodAngle(
                Math.toDegrees(shotSolution.launchAngleRad())
            );
        }
    }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));

        Logger.recordOutput("Shot/Trajectory3d", new Pose3d[0]);

        calculated = false;
    }
}
