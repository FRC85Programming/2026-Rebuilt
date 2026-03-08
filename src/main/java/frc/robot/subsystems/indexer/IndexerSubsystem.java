package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

public class IndexerSubsystem extends SubsystemBase {

    // Indexer - The spinny thing in the center of the robot
    private final SparkFlex indexerMotor = 
        new SparkFlex(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    // Feed - The green compliant wheel that kicks the ball upwards
    private final SparkFlex feederMotor =
        new SparkFlex(ShooterConstants.FEED_MOTOR_ID, MotorType.kBrushless);

    // Belt - The belt that feeds the ball into the feeder
    private final SparkFlex beltMotor =
        new SparkFlex(ShooterConstants.BELT_MOTOR_ID, MotorType.kBrushless);

    // TODO: Tune these values
    // Speeds that each system runs to create an ideal path. These should all be positive
    private double indexSpeed = 0.5;
    private double beltSpeed = 0.5;
    private double feederSpeed = 0.5;


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
     * Sets the speed of the feeder. This WILL NOT feed balls all the way through the system. This only sets the speed of 
     * the roller part of the indexer.
     * 
     * @param speed The speed the feeder roller runs at. Positive speed feeds the ball towards the shooter
     */
    public void setFeederSpeed(double speed) {
        // TODO: Figure out what speed feeds inwards
        feederMotor.set(speed);
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
        setIndexerSpeed(indexSpeed);
        setBeltSpeed(beltSpeed);
        setFeederSpeed(feederSpeed);
    }

    /**
     * Stops all parts of the indexer
     */
    public void stopIndexing() {
        setIndexerSpeed(0);
        setBeltSpeed(0);
        setFeederSpeed(0);
    }
}
