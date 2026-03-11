package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex indexerMotor = 
        new SparkFlex(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex beltMotor =
        new SparkFlex(IndexerConstants.BELT_MOTOR_ID, MotorType.kBrushless);

    private double indexSpeed = 0.5;
    private double beltSpeed = 0.5;

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
    }


    /**
     * Sets the speed of the indexer. This WILL NOT feed balls all the way through the system. This only sets the speed of 
     * the spinny spart of the indexer.
     * 
     * @param speed The spinny part of the indexer runs at. Positive speed feeds the ball towards the shooter
     */
    public void setIndexerSpeed(double speed) {
        // TODO: Figure out what speed feeds inwards
        indexerMotor.set(speed);
    }

    /**
     * Sets the speed of the belt part of the indexer
     * 
     * @param speed The speed the belt runs at. Positive speed feeds the ball towards the shooter
     */
    public void setBeltSpeed(double speed) {
        // TODO: Figure out what speed feeds inwards
        beltMotor.set(speed);
    }

    /**
     * Starts all the parts of the indexer at their tuned speeds
     */
    public void startIndexing() {
                // TODO: Change back

        // setIndexerSpeed(indexSpeed);
        // setBeltSpeed(beltSpeed);
    }

    /**
     * Stops all parts of the indexer
     */
    public void stopIndexing() {
        // setIndexerSpeed(0);
        // setBeltSpeed(0);
    }
}
