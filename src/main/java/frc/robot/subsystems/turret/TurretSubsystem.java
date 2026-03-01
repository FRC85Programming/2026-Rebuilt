package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
    private final SparkFlex turretMotor =
        new SparkFlex(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

    private final DutyCycleEncoder turretEncoder = new DutyCycleEncoder(1);

    private final TurretSim turretSim = new TurretSim();

    private final boolean isSim;
    double absEncoderHome = 0.14046377851159447;
    
    private Supplier<Double> autoAngleSupplier = null;

    SparkClosedLoopController closedLoopController;

    SparkMaxConfig turretConfig = new SparkMaxConfig();

    private double goalAngle = 0;

    public TurretSubsystem() {
        isSim = Robot.isSimulation();

        closedLoopController = turretMotor.getClosedLoopController();

        turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.35)
                .i(0)
                .d(0)
                .outputRange(-0.25, 0.25)
                .positionWrappingEnabled(false)
                .feedForward.kV(12.0 / 6784);

        turretConfig.idleMode(IdleMode.kBrake);

        turretConfig.inverted(true);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //turretMotor.getEncoder().setPosition(((turretEncoder.get() - absEncoderHome)) * TurretConstants.TURRET_GEAR_RATIO);

        turretMotor.getEncoder().setPosition(0);
        SmartDashboard.putNumber("Turret P", 1);
        SmartDashboard.putNumber("Setpoint", 0);
    }

    @Override
    public void periodic() {
        updateLogging();
        if (RobotBase.isSimulation()) {
            // TODO: Sim still needs PID so figure out how to do that I guess
            //turretSim.update(turretOut * 12.0, 0.02);

                AlphaMechanism3d.setTurretAngle(
                isSim ? turretSim.getTurretAngleRads() : getTurretAngleRads()
            );
        }
    }

    public void setTurretAngle(double angleDeg) {
        angleDeg = MathUtil.inputModulus(angleDeg, -180.0, 180.0);

        if (angleDeg > TurretConstants.TURRET_UPPER_LIMIT_DEG) {
            angleDeg -= 360.0;
        } else if (angleDeg < TurretConstants.TURRET_LOWER_LIMIT_DEG) {
            angleDeg += 360.0;
        }

        angleDeg = MathUtil.clamp(angleDeg, TurretConstants.TURRET_LOWER_LIMIT_DEG, TurretConstants.TURRET_UPPER_LIMIT_DEG);

        goalAngle = (angleDeg / 360.0) * TurretConstants.TURRET_GEAR_RATIO;
        closedLoopController.setSetpoint(goalAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("Turret Goal Angle", angleDeg);
    }

    public double getTurretAngleRads() {
        if (RobotBase.isSimulation()) {
            return turretSim.getTurretAngleRads();
        }
        return (turretMotor.getEncoder().getPosition() / TurretConstants.TURRET_GEAR_RATIO) * (Math.PI*2);
    }
    
    public void setAutoAngleSupplier(Supplier<Double> angleSupplier) {
        this.autoAngleSupplier = angleSupplier;
    }
    
    public void clearAutoAngleSupplier() {
        this.autoAngleSupplier = null;
    } 

    public void setTurretSpeed(double speed) {
        turretMotor.set(speed);
    }

    public void updateLogging() {
        SmartDashboard.putNumber("Motor Setpoint", closedLoopController.getSetpoint());
        SmartDashboard.putNumber("Turret Angle Measured (deg)", Math.toDegrees(getTurretAngleRads()));
        SmartDashboard.putNumber("Turret ABS Encoder Value", turretEncoder.get());
        SmartDashboard.putNumber("Turret Motor Encoder", turretMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Turret RAD", getTurretAngleRads());
        SmartDashboard.putNumber("Turret DEG", Math.toDegrees(getTurretAngleRads()));
    }

    public boolean turretAtAngle(double tolerance) {
        double currentDeg = Math.toDegrees(getTurretAngleRads());
        double goalDeg = (goalAngle / TurretConstants.TURRET_GEAR_RATIO) * 360.0;
        return Math.abs(currentDeg - goalDeg) < tolerance;
    }
}
