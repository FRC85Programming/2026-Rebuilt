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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    // Roller Motor - The motor that turns the roller part of the intake
    private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    // Pivot Motor - The motor that controlls the up and down motion of the intake
    private final SparkFlex pivotMotor = new SparkFlex(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    // The config for using position control on the pivot
    private SparkMaxConfig pivotConfig = new SparkMaxConfig();

    // The actual controller where setpoints are passed to
    private SparkClosedLoopController pivotController;

    // The defined optimal speed for the intake rollers
    private double rollerSpeed = 0.5;

    public IntakeSubsystem() {
        pivotController = pivotMotor.getClosedLoopController();

        // The configuration for closed loop positon control of the pivot
        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.35)
                .i(0)
                .d(0)
                // Output clamp range
                .outputRange(-0.6, 0.6)
                // Not a >360 degree system. Disable position wrapping
                .positionWrappingEnabled(false)
                // Battery voltage divided by RPM max = feedforward
                .feedForward.kV(12.0 / 6784);

        // Keep brake mode enabled
        pivotConfig.idleMode(IdleMode.kBrake);

        // Set the config from before to the motor
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Set the pivot's position to 0 (assumes pivot always starts up)
        pivotMotor.getEncoder().setPosition(0);
    }

    /**
     * Runs the intake rollers at a predefined optimal speed
     */
    public void runRollers() {
        rollerMotor.set(rollerSpeed);
    }

    /**
     * Stops the intake rollers
     */
    public void stopRollers() {
        rollerMotor.set(0);
    }

    /**
     * Deploys the intake to the downwards position using closed loop control
     */
    public void deployIntake() {
        pivotController.setSetpoint(IntakeConstants.INTAKE_DOWN_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /**
     * Retracts the intake into the robot using closed loop control
     */
    public void retractIntake() {
        // Setpoint is 0 as a home position
        pivotController.setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
}
