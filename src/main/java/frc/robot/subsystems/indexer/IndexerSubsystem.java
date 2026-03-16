package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex indexerMotor =
        new SparkFlex(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex beltMotor =
        new SparkFlex(IndexerConstants.BELT_MOTOR_ID, MotorType.kBrushless);

    private double indexSpeed = -0.3;
    private double beltSpeed = -0.5;

    private double agitateForwardSpeed = -0.2;
    private double agitateReverseSpeed = 0.1;

    private double agitateForwardDuration = 0.4;
    private double agitateReverseDuration = 0.15;

    private final Timer agitationTimer = new Timer();
    private boolean agitatingForward = true;
    private boolean agitationEnabled = false;

    public IndexerSubsystem() {
        SparkFlexConfig indexerConfig = new SparkFlexConfig();
        indexerConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(500)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        SparkFlexConfig beltConfig = new SparkFlexConfig();
        beltConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(500)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        beltMotor.configure(beltConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putNumber("Index Speed", indexSpeed);
        SmartDashboard.putNumber("Belt Speed", beltSpeed);
        SmartDashboard.putNumber("Agitate Forward Speed", agitateForwardSpeed);
        SmartDashboard.putNumber("Agitate Reverse Speed", agitateReverseSpeed);
        SmartDashboard.putNumber("Agitate Forward Duration", agitateForwardDuration);
        SmartDashboard.putNumber("Agitate Reverse Duration", agitateReverseDuration);

        agitationTimer.start();
    }

    @Override
    public void periodic() {
        indexSpeed             = SmartDashboard.getNumber("Index Speed", indexSpeed);
        beltSpeed              = SmartDashboard.getNumber("Belt Speed", beltSpeed);
        agitateForwardSpeed    = SmartDashboard.getNumber("Agitate Forward Speed", agitateForwardSpeed);
        agitateReverseSpeed    = SmartDashboard.getNumber("Agitate Reverse Speed", agitateReverseSpeed);
        agitateForwardDuration = SmartDashboard.getNumber("Agitate Forward Duration", agitateForwardDuration);
        agitateReverseDuration = SmartDashboard.getNumber("Agitate Reverse Duration", agitateReverseDuration);

        if (agitationEnabled && Math.abs(indexerMotor.getAppliedOutput()) < 0.01) {
            runAgitation();
        }
    }

    /**
     * Sets the speed of the indexer. This WILL NOT feed balls all the way through the system.
     * This only sets the speed of the spinny part of the indexer.
     *
     * @param speed The speed the spinny part of the indexer runs at. Positive speed feeds the ball towards the shooter.
     */
    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }

    /**
     * Sets the speed of the belt part of the indexer.
     *
     * @param speed The speed the belt runs at. Positive speed feeds the ball towards the shooter.
     */
    public void setBeltSpeed(double speed) {
        beltMotor.set(speed);
    }

    /**
     * Starts all parts of the indexer at their tuned feed speeds.
     */
    public void startIndexing() {
        indexerMotor.set(indexSpeed);
        beltMotor.set(beltSpeed);
    }

    /**
     * Stops commanded motion. Agitation remains enabled, so passive
     * agitation will resume on the next periodic() cycle if the motor
     * output drops to zero.
     */
    public void stopIndexing() {
        indexerMotor.set(0);
        beltMotor.set(0);
    }

    /**
     * Rocks the indexer back and forth to keep balls loose.
     * Enables passive agitation and drives the motor this cycle.
     * Belt stays still — only the agitator moves.
     * Call this every cycle when no active feed is needed.
     */
    public void runAgitation() {
        agitationEnabled = true;

        double elapsed = agitationTimer.get();
        double phaseDuration = agitatingForward ? agitateForwardDuration : agitateReverseDuration;

        if (elapsed >= phaseDuration) {
            agitatingForward = !agitatingForward;
            agitationTimer.reset();
        }

        indexerMotor.set(agitatingForward ? agitateForwardSpeed : agitateReverseSpeed);
    }

    /**
     * Hard stops all indexer motors and disables passive agitation.
     * Agitation will not resume until runAgitation() is called again.
     */
    public void disableIndexer() {
        agitationEnabled = false;
        indexerMotor.set(0);
        beltMotor.set(0);
    }
}