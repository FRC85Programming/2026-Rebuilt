package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex pivotMotor = new SparkFlex(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    private SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private SparkFlexConfig rollerConfig = new SparkFlexConfig();

    private SparkClosedLoopController pivotController;

    private double rollerSpeed = 0.5;

    public IntakeSubsystem() {
        pivotController = pivotMotor.getClosedLoopController();

        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.01)
                .i(0)
                .d(0)
                .outputRange(-0.25, 0.25)
                .positionWrappingEnabled(false)
                .feedForward.kV(12.0 / 6784);

        pivotConfig.idleMode(IdleMode.kBrake);

        pivotConfig.signals
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(100)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        rollerConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(500)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        pivotConfig.idleMode(IdleMode.kBrake);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotMotor.getEncoder().setPosition(0);
    }

    /**
     * Runs the intake rollers at a predefined optimal speed
     */
    public void runRollers() {
        // TODO: Change back
        //rollerMotor.set(rollerSpeed);
    }

    /**
     * Stops the intake rollers
     */
    public void stopRollers() {
        //rollerMotor.set(0);
    }

    /**
     * Deploys the intake to the downwards position using closed loop control
     */
    public void deployIntake() {
        //pivotController.setSetpoint(IntakeConstants.INTAKE_DOWN_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /**
     * Retracts the intake into the robot using closed loop control
     */
    public void retractIntake() {
        // Setpoint is 0 as a home position
        pivotController.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
