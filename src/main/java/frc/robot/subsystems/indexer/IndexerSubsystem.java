package frc.robot.subsystems.indexer;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;


public class IndexerSubsystem extends SubsystemBase {
    private final static SparkFlex indexerMotor = 
            new SparkFlex(IndexerConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
        
            public void index(double speed){
                indexerMotor.set(speed);
    }

}