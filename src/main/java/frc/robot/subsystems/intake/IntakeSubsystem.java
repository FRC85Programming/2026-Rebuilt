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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum PivotState {
        /** Intake fully deployed for collecting game pieces. */
        DOWN,
        /** Intake partially raised inside the robot frame. */
        STOWED,
        /** Intake fully retracted to home position. */
        UP,
        /** Intake partially down to intake from the depot */
        DEPOT
    }

    private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex pivotMotor = new SparkFlex(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);

    private SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private SparkFlexConfig rollerConfig = new SparkFlexConfig();

    private SparkClosedLoopController pivotController;

    private double rollerSpeed = -0.55;

    private PivotState pivotState = PivotState.STOWED;

    public IntakeSubsystem() {
        pivotController = pivotMotor.getClosedLoopController();

        pivotConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.35)
                .i(0)
                .d(0)
                .outputRange(-0.15, 0.15)
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
            .faultsPeriodMs(500
            );

        rollerConfig.signals
            .primaryEncoderPositionPeriodMs(500)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(500)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        pivotConfig.idleMode(IdleMode.kBrake);

        rollerConfig.idleMode(IdleMode.kCoast);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotMotor.getEncoder().setPosition(-9.4);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putString("Intake Pivot State", pivotState.toString());
    }

    /**
     * Sets the pivot state and commands the motor to the corresponding position.
     */
    public void setPivotState(PivotState state) {
        pivotState = state;
        switch (state) {
            case DOWN:
                pivotController.setSetpoint(IntakeConstants.INTAKE_DOWN_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                break;
            case STOWED:
                pivotController.setSetpoint(IntakeConstants.INTAKE_STOW_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                break;
            case UP:
                pivotController.setSetpoint(IntakeConstants.INTAKE_UP_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                break;
            case DEPOT:
                 pivotController.setSetpoint(IntakeConstants.INTAKE_DEPOT_POSITION, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                 break;
        }
    }

    /**
     * Returns the current pivot state.
     */
    public PivotState getPivotState() {
        return pivotState;
    }

    /**
     * Toggles the pivot between STOWED and UP.
     * If currently UP, moves to STOWED. Otherwise, moves to UP.
     */
    public void toggleStowedUp() {
        if (pivotState == PivotState.STOWED) {
            setPivotState(PivotState.UP);
            stopRollers();
        } else {
            setPivotState(PivotState.STOWED);
        }
    }

    /**
     * Runs the intake rollers at a predefined optimal speed
     */
    public void runRollers() {
        // TODO: Change back
        rollerMotor.set(rollerSpeed);
    }

    /**
     * Stops the intake rollers
     */
    public void stopRollers() {
        rollerMotor.set(0);
    }

    public void reverseRollers() {
        rollerMotor.set(-rollerSpeed);
    }

    /**
     * Deploys the intake to the downwards position using closed loop control
     */
    public void deployIntake() {
        setPivotState(PivotState.DOWN);
    }

    /**
     * Retracts the intake into the robot using closed loop control
     */
    public void retractIntake() {
        setPivotState(PivotState.UP);
    }

    public void stowIntake() {
        setPivotState(PivotState.STOWED);
    }

    public void deployIntakeDepot() {
        setPivotState(PivotState.DEPOT);
    }

    public boolean isPivotAtSetpoint(double tolerance) {
        return pivotController.isAtSetpoint();
    }
}
