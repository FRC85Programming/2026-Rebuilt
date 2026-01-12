package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.ShotSolver;
import frc.robot.util.ShotSolver.ShotSolution;

public class Shoot extends Command{
    ShooterSubsystem shooter;
    SwerveSubsystem swerve;
    ShotSolution shotSolution;
    Translation2d targetTranslation = new Translation2d(8.25, 4); // 2022 hub location

    public Shoot(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        // Placeholder values for testing
        // There seems to be trouble with getting valid solutions with high drop angles
        // We should make a tool that solves for the lowest drop angle
        var solution = ShotSolver.solve(
            swerve.getPose().getTranslation().getDistance(targetTranslation), 
            0.305,
            1.7272,
            Math.toRadians(50), 
            Math.toRadians(Constants.ShooterConstants.HOOD_MIN_ANGLE),
            Math.toRadians(Constants.ShooterConstants.HOOD_MAX_ANGLE)
        );

        if (solution.isEmpty()) {
            SmartDashboard.putString("ShotSolver Status", "NO VALID SOLUTION");
            return;
        }

        shotSolution = solution.get();

        SmartDashboard.putNumber(
            "Calculated Shot Angle (deg)",
            Math.toDegrees(shotSolution.launchAngleRad())
        );

        SmartDashboard.putNumber(
            "Calculated Shot Speed (mps)",
            shotSolution.launchVelocityMps()
        );

        SmartDashboard.putNumber(
            "Distance To Target",
            swerve.getPose().getTranslation().getDistance(targetTranslation)
        );

        SmartDashboard.putNumber(
            "Calculated Shot Speed (rpm)",
            shooter.mpsToRPM(shotSolution.launchVelocityMps())
        );
    }

    @Override
    public void execute() {
        swerve.aimAtPosition(targetTranslation);
        shooter.setFlywheelRPM(shooter.mpsToRPM(shotSolution.launchVelocityMps()));
        shooter.setHoodAngle(Math.toDegrees(shotSolution.launchAngleRad()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
}
