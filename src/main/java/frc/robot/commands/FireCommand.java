package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class FireCommand extends Command {

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    private final Timer intakeTimer = new Timer();
    private boolean intakeIsDown = false;

    public FireCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.turret = turret;
        this.indexer = indexer;
        this.intake = intake;
        addRequirements(shooter, turret);
    }

    @Override
    public void initialize() {
        intakeTimer.restart();
        intakeIsDown = false;
        if (intake.getCurrentCommand() == null) {
            intake.stowIntake();
        }
    }

    @Override
    public void execute() {
        shooter.setFlywheelRPM(shooter.getCalculatedRPM());

        boolean ready = shooter.flywheelAtSpeed(0.80) && shooter.hoodAtAngle(1) && turret.turretAtAngle(4);
        SmartDashboard.putBoolean("AIMED", ready);
        SmartDashboard.putBoolean("READY FLYWHEEL", shooter.flywheelAtSpeed(0.80));
        SmartDashboard.putBoolean("READY HOOD", shooter.hoodAtAngle(1));
        SmartDashboard.putBoolean("READY TURRET", turret.turretAtAngle(2));

        if (ready) {
            indexer.startIndexing();
        } else {
            indexer.runAgitation(); 
        }

        if (intake.getCurrentCommand() == null) {
            if (intakeTimer.advanceIfElapsed(0.65)) {
                intakeIsDown = !intakeIsDown;
                if (intakeIsDown) intake.deployIntake();
                else              intake.stowIntake();
            }
        } else {
            intakeTimer.restart();
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
        intake.stowIntake();
    }
}