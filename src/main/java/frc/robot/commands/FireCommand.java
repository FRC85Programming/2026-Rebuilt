package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Spins the flywheel to the RPM calculated by the shooter subsystem's active state
 * (AIMING or FEEDING) and releases the feed mechanism once all systems are on target.
 * State management (which target and lookup table to use) is handled externally in
 * RobotContainer via startAiming() / startIndexing() calls.
 */
public class FireCommand extends Command {

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final IndexerSubsystem indexer;

    public FireCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret) {
        this.shooter = shooter;
        this.turret = turret;
        this.indexer = indexer;
        addRequirements(shooter, turret);
    }

    @Override
    public void execute() {
        shooter.setFlywheelRPM(shooter.getCalculatedRPM());

        boolean ready = shooter.flywheelAtSpeed(0.80) && shooter.hoodAtAngle(1) && turret.turretAtAngle(1);
        SmartDashboard.putBoolean("AIMED", ready);
        SmartDashboard.putBoolean("READY FLYWHEEL", shooter.flywheelAtSpeed(0.80));
        SmartDashboard.putBoolean("READY HOOD", shooter.hoodAtAngle(1));
        SmartDashboard.putBoolean("READY TURRET", turret.turretAtAngle(1));

        if (ready) {
            indexer.startIndexing();
        } else {
            indexer.stopIndexing();
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        indexer.stopIndexing();
    }
}
