package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem.Animation;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;

public class FireCommand extends Command {

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final LEDSubsystem leds;

    private final Timer intakeTimer = new Timer();
    private boolean intakeIsDown = false;
    private double turretTolerance = 9;

    public FireCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret, IntakeSubsystem intake, LEDSubsystem leds) {
        this.shooter = shooter;
        this.turret = turret;
        this.indexer = indexer;
        this.intake = intake;
        this.leds = leds;
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

        if (turret.getState() == TurretState.FEEDING) {
            turretTolerance = SmartDashboard.getNumber("FEED TOLERANCE", 25);
        } else {
            turretTolerance = SmartDashboard.getNumber("TOLERANCE", 9);
        }
        boolean ready = shooter.flywheelAtSpeed(0.95) && shooter.hoodAtAngle(1) && turret.turretAtAngle(turretTolerance);
        SmartDashboard.putBoolean("AIMED", ready);
        SmartDashboard.putBoolean("READY FLYWHEEL", shooter.flywheelAtSpeed(0.80));
        SmartDashboard.putBoolean("READY HOOD", shooter.hoodAtAngle(1));
        SmartDashboard.putBoolean("READY TURRET", turret.turretAtAngle(turretTolerance));

        if (ready && Math.abs(Math.toDegrees(turret.getTurretAngleRads()) - Math.toDegrees(turret.getTurretSetpointRadians())) < 45) {
            indexer.startIndexing();
            leds.setAnimation(Animation.BLINK_GREEN);
        } else {
            indexer.runAgitation(); 
            leds.setAnimation(Animation.BLINK_WHITE);
        }

        SmartDashboard.putNumber("Turret Setpoint Degrees", Math.toDegrees(turret.getTurretSetpointRadians()));

        if (intake.getCurrentCommand() == null) {
            if (intakeTimer.advanceIfElapsed(0.45)) {
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
        if (!DriverStation.isAutonomous()) {
            leds.setAnimation(Animation.ALLIANCE_SPECIFIC);
        } else {
            leds.setAnimation(Animation.AUTO);
        }
    }
}