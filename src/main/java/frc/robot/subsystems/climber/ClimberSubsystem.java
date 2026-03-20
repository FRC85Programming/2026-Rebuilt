package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkFlex climbMotor = new SparkFlex(ClimberConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
    SparkClosedLoopController closedLoopController;
    SparkMaxConfig climbConfig = new SparkMaxConfig();

    public ClimberSubsystem() {
        closedLoopController = climbMotor.getClosedLoopController();

        climbConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.5)
                .i(0)
                .d(0)
                .outputRange(-0.8, 0.8)
                .positionWrappingEnabled(false)
                .feedForward.kV(12.0 / 6784);

        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climbMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", climbMotor.getEncoder().getPosition());
    }

    public void climberUp() {
        closedLoopController.setSetpoint(ClimberConstants.CLIMBER_UP_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void climberDown() {
        closedLoopController.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
