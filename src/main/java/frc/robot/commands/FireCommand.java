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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.TurretState;
import swervelib.SwerveDrive;

public class FireCommand extends Command {

    private final SwerveSubsystem swerve;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final LEDSubsystem leds;

    private final Timer intakeTimer = new Timer();
    private boolean intakeIsDown = false;
    private double turretTolerance = 9;
    boolean ready = false;

    public FireCommand(SwerveSubsystem swerve, ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret, IntakeSubsystem intake, LEDSubsystem leds) {
        this.swerve = swerve;
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

        if (turret.getState() == TurretState.AIMING) {
            swerve.setSpeedMultiplier(0.25);
        }
    }

    @Override
    public void execute() {
        shooter.setFlywheelRPM(shooter.getCalculatedRPM());

        if (turret.getState() == TurretState.FEEDING) {
            ready = shooter.flywheelAtSpeed(0.60);
        } else {
            ready = shooter.flywheelAtSpeed(0.95) && shooter.hoodAtAngle(1) && turret.turretAtAngle(turret.getMaxTurretError());
        }

        SmartDashboard.putBoolean("AIMED", ready);
        SmartDashboard.putBoolean("READY FLYWHEEL", shooter.flywheelAtSpeed(0.95));
        SmartDashboard.putBoolean("READY HOOD", shooter.hoodAtAngle(1));
        SmartDashboard.putBoolean("READY TURRET", turret.turretAtAngle(turret.getMaxTurretError()));

        if (ready && turret.isSpeedSafeToFire()) {

            indexer.startIndexing();

            // TODO: Test to see if this improves fire rate
            //indexer.indexAtProportionalRate(shooter.getFlywheelErrorPercentage());

            if (turret.getState() == TurretState.AIMING) {
                leds.setAnimation(Animation.BLINK_GREEN);
            } else if (turret.getState() == TurretState.FEEDING) {
                leds.setAnimation(Animation.BLINK_GREEN);
            } else if (turret.getState() == TurretState.MANUALSHOOT) {
                leds.setAnimation(Animation.BLINK_GREEN);
            }
        } else {
            indexer.runAgitation(); 
            leds.setAnimation(Animation.BLINK_WHITE);
        }

        SmartDashboard.putNumber("Turret Setpoint Degrees", Math.toDegrees(turret.getTurretSetpointRadians()));

        if (intake.getCurrentCommand() == null) {
            if (intakeTimer.advanceIfElapsed(0.8)) {
                intakeIsDown = !intakeIsDown;

                /** TODO: Try different wheel running configs when agitating
                 * so that balls dont fly everywhere (wheels off, slow wheels) */
                if (!intakeIsDown) {
                    intake.stowIntake();
                    intake.stopRollers();
                } else {
                    intake.deployIntake();
                    intake.runRollers();
                }
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
        swerve.setSpeedMultiplier(1);
        if (!DriverStation.isAutonomous()) {
            leds.setAnimation(Animation.ALLIANCE_SPECIFIC);
        } else {
            leds.setAnimation(Animation.AUTO);
        }  
    }
}