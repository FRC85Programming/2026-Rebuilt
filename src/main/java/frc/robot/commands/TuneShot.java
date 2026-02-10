package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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

    public TuneShot(SwerveSubsystem swerve, ShooterSubsystem shooter, Supplier<Translation3d> target) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;

        SmartDashboard.putBoolean("Feed", false);
        SmartDashboard.putNumber("Feed Speed", -0.8);
    }

    private void calculateSolution() {
        targetTranslation = target.get();

        // Distance from SHOOTER to hub
        Translation2d toTarget2d =
            targetTranslation.toTranslation2d()
                .minus(swerve.getPose().getTranslation().plus(ShooterConstants.SHOOTER_TRANSLATION));

        double distance = toTarget2d.getNorm();

        SmartDashboard.putNumber("TUNE Distance To Hub", distance);

        // Protect against super short distances/pose glitches
        if (distance < 1e-6) {
            return;
        }

            
        goalRPM = SmartDashboard.getNumber("TUNE Shot RPM", 0);
        goalAngle = SmartDashboard.getNumber("TUNE Shot Angle", 80);


        swerve.aimAtPositionWithLead(targetTranslation.toTranslation2d(), 0, false);
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
            shooter.setFeedSpeed(SmartDashboard.getNumber("Feed Speed", -0.8));
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }


    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
    }
}
