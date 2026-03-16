package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;


public class Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final Supplier<Translation3d> target;

    public Shoot(SwerveSubsystem swerve, ShooterSubsystem shooter, TurretSubsystem turret, Supplier<Translation3d> target) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
        this.turret = turret;

        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        shooter.startAiming(swerve, target);
        turret.startAiming(swerve, target);
    }

    @Override
    public void execute() {
        // Subsystems handle all aiming math in their own periodic().
        // Here we only spin the flywheel to the speed the shooter subsystem calculated.
        shooter.setFlywheelRPM(shooter.getCalculatedRPM());

        boolean ready = shooter.flywheelAtSpeed(0.80) && shooter.hoodAtAngle(1) && turret.turretAtAngle(1);
        SmartDashboard.putBoolean("AIMED", ready);
        SmartDashboard.putBoolean("READY FLYWHEEL", shooter.flywheelAtSpeed(0.80));
        SmartDashboard.putBoolean("READY HOOD", shooter.hoodAtAngle(1));
        SmartDashboard.putBoolean("READY TURRET", turret.turretAtAngle(1));

        if (ready) {
            shooter.setFeedSpeed(0.5);
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
    }
}
