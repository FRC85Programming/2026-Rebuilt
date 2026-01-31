package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.TargetingCalculator;

public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final Supplier<Translation3d> target;

    private TargetingCalculator.TargetingSolution currentSolution;

    private final boolean isPathPlanner;

    public Shoot(
        SwerveSubsystem swerve,
        ShooterSubsystem shooter,
        TurretSubsystem turret,
        Supplier<Translation3d> target,
        boolean isPathPlanner
    ) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.turret = turret;
        this.target = target;
        this.isPathPlanner = isPathPlanner;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.cleanupPieces();
        swerve.resetAimController();
    }

    @Override
    public void execute() {
        currentSolution = TargetingCalculator.calculateShot(
            target.get(),
            swerve.getPose(),
            swerve.getFieldVelocity(),
            turret.getTurretAngleRads(),
            new TargetingCalculator.RpmConverter() {
                public double rpmToMps(double rpm) {
                    return shooter.rpmToMps(rpm);
                }
                public double mpsToRpm(double mps) {
                    return shooter.mpsToRpm(mps);
                }
            }
        );

        shooter.setFlywheelRPM(currentSolution.flywheelRPM);
        shooter.setHoodAngle(currentSolution.hoodAngleDegrees);
        turret.setAngle(currentSolution.turretAngleDegrees);

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
            && turret.atGoal()
            && swerve.isAimedAtPosition(0.1)
        ) {
            if (shooter.generateProjectileIsReady()) {
                shooter.simulatedShot(
                    swerve.getPose(),
                    swerve.getFieldVelocity(),
                    new Rotation2d(turret.getTurretAngleRads())
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
