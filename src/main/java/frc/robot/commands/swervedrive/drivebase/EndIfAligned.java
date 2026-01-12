package frc.robot.commands.swervedrive.drivebase;

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

public class EndIfAligned extends Command{
    SwerveSubsystem swerve;
    Translation2d targetTranslation = new Translation2d(4.620, 4.030); // Rebuilt BLUE goal



    public EndIfAligned(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("EndIfAligned", "Started");
    }


    @Override
    public boolean isFinished() {
        return swerve.isAimedAtPosition(0.03);
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("EndIfAligned", "Ended");
    }
}
