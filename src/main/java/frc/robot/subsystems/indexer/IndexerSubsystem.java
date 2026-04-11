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

    public enum IndexerState {
        /** Actively feeding balls toward the shooter. */
        RUN,
        /** Rocking the indexer wheel back and forth to prevent jams. */
        AGITATE,
        /** All motors stopped; no agitation. */
        STOP
    }

    private final SparkFlex indexerMotor =
        new SparkFlex(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex beltMotor =
        new SparkFlex(IndexerConstants.BELT_MOTOR_ID, MotorType.kBrushless);

    private double indexSpeed = -0.6;
    private double beltSpeed = -0.9;

    private double agitateForwardSpeed = -0.2;
    private double agitateReverseSpeed = 0.1;

    private double agitateForwardDuration = 0.4;
    private double agitateReverseDuration = 0.15;

    private final Timer agitationTimer = new Timer();
    private boolean agitatingForward = true;

    private IndexerState state = IndexerState.STOP;

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

        SmartDashboard.putString("Indexer State", state.name());

        if (state == IndexerState.AGITATE) {
            performAgitation();
        }
    }

    private void performAgitation() {
        double elapsed = agitationTimer.get();
        double phaseDuration = agitatingForward ? agitateForwardDuration : agitateReverseDuration;

        if (elapsed >= phaseDuration) {
            agitatingForward = !agitatingForward;
            agitationTimer.reset();
        }

        indexerMotor.set(agitatingForward ? agitateForwardSpeed : agitateReverseSpeed);
    }

    /**
     * Sets the speed of the indexer wheel directly, transitioning to the RUN state
     * so agitation does not interfere with external motor control.
     *
     * @param speed The speed of the indexer wheel. Positive feeds toward the shooter.
     */
    public void setIndexerSpeed(double speed) {
        state = IndexerState.RUN;
        indexerMotor.set(speed);
    }

    /**
     * Sets the speed of the belt directly, transitioning to the RUN state
     * so agitation does not interfere with external motor control.
     *
     * @param speed The speed of the belt. Positive feeds toward the shooter.
     */
    public void setBeltSpeed(double speed) {
        state = IndexerState.RUN;
        beltMotor.set(speed);
    }

    /**
     * Transitions to the RUN state and drives both motors at their tuned feed speeds.
     */
    public void startIndexing() {
        state = IndexerState.RUN;
        indexerMotor.set(indexSpeed);
        beltMotor.set(beltSpeed);
    }

    /**
     * Transitions to the AGITATE state. The belt stops immediately and the indexer
     * wheel will be rocked back and forth by periodic() to keep balls loose.
     */
    public void stopIndexing() {
        enterAgitateState();
        beltMotor.set(0);
    }

    /**
     * Transitions to the AGITATE state so the indexer wheel rocks back and forth
     * while no active feed is needed. The belt is unaffected.
     */
    public void runAgitation() {
        enterAgitateState();
    }

    /**
     * Transitions to the STOP state, immediately halting all motors.
     * Agitation will not run until the state changes.
     */
    public void disableIndexer() {
        state = IndexerState.STOP;
        indexerMotor.set(0);
        beltMotor.set(0);
    }

    /** Returns the current indexer state. */
    public IndexerState getState() {
        return state;
    }

    private void enterAgitateState() {
        if (state != IndexerState.AGITATE) {
            agitatingForward = true;
            agitationTimer.reset();
        }
        state = IndexerState.AGITATE;
    }

    // Something to try
    public void indexAtProportionalRate(double proportion) {
        // Square function
        indexerMotor.set(indexSpeed * (proportion * proportion));
        beltMotor.set(beltSpeed);
    }
}