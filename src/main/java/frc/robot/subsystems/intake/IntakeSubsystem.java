package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }
}
