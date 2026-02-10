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
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.BallisticTrajectory;
import frc.robot.util.BallisticTrajectory3d;
import frc.robot.util.ShooterTable;
import frc.robot.util.ShotSolver;
import frc.robot.util.ShotSolver.ShotSolution;
import frc.robot.util.TrajectoryTransform3d;

public class Shoot extends Command{
    ShooterSubsystem shooter;
    SwerveSubsystem swerve;
    Supplier<Translation3d> target;
    List<Translation3d> fieldTrajectory3d;
    double goalRPM = 0;
    double goalAngle = 0;
    boolean isPathPlanner;
    Translation3d targetTranslation;

    public Shoot(SwerveSubsystem swerve, ShooterSubsystem shooter, Supplier<Translation3d> target, boolean isPathPlaner) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
        this.isPathPlanner = isPathPlaner;
    }

    @Override
    public void initialize()
    {
        shooter.cleanupPieces();    
    }

    private void calculateSolution() {
        targetTranslation = target.get();
        
        // Convert the field relative velocity into velocity relative to the goal
        ChassisSpeeds robotVel = swerve.getFieldVelocity();
        Translation2d toTarget2d =
            targetTranslation.toTranslation2d()
                .minus(swerve.getPose().getTranslation());

        double distance = toTarget2d.getNorm();

        SmartDashboard.putNumber("Distance From Hub", distance);

        // Protect against super short distances/pose glitches
        if (distance < 1e-6) {
            return;
        }

        Translation2d toTargetUnit = toTarget2d.div(distance);
        double robotVx = 
            robotVel.vxMetersPerSecond * toTargetUnit.getX() 
            + robotVel.vyMetersPerSecond * toTargetUnit.getY();

            
        goalRPM = ShooterTable.getSetpoint(distance).flywheelRPM();
        goalAngle = ShooterTable.getSetpoint(distance).hoodAngle().getRadians();

        // Calculate new speeds based of robot velocity (this should be commented out on first real robot test)
        double shotSpeed = shooter.rpmToMps(goalRPM);
        double timeOfFlight = distance / shotSpeed;
        double compensatedShotSpeed = shotSpeed - robotVx;
        goalRPM = shooter.mpsToRpm(compensatedShotSpeed);

        double compensatedDistance = distance - robotVx * timeOfFlight;
        goalAngle = ShooterTable.getSetpoint(compensatedDistance).hoodAngle().getRadians();

        // Calculate lead angle based on velocity (this should be commented out on first real robot test)
        double robotVy =
            -robotVel.vxMetersPerSecond * toTargetUnit.getY()
            + robotVel.vyMetersPerSecond * toTargetUnit.getX();

        double lead = robotVy * timeOfFlight;

        swerve.aimAtPositionWithLead(targetTranslation.toTranslation2d(), -lead, isPathPlanner);

        // Create trajectories for sim visulization (could be wrapped in a sim check if the code runs slow) - this does not affect the shots at all
        var shooterRelativeTrajectory =
            BallisticTrajectory3d.generate(
                shooter.rpmToMps(goalRPM),
                goalAngle,
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

        shooter.setFlywheelRPM(
            goalRPM
        );
    
        shooter.setHoodAngle(
            Math.toDegrees(goalAngle)
        );
        SmartDashboard.putBoolean("Flywheel at speed", shooter.flywheelAtSpeed(200));
        SmartDashboard.putBoolean("Hood at angle", shooter.hoodAtAngle(3));


        if (shooter.flywheelAtSpeed(200) && shooter.hoodAtAngle(3) && swerve.isAimedAtPosition(0.1)) {
                if (shooter.generateProjectileIsReady()) {
                    shooter.simulatedShot(swerve.getPose(), swerve.getFieldVelocity());
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
