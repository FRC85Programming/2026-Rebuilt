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

    private final ProfiledPIDController angleController =
        new ProfiledPIDController(
            0.07,  
            0.0,   
            0.0,   
            new TrapezoidProfile.Constraints(
                180.0,
                360.0 
            )
        );

    private double goalAngleDeg = 0.0;
    private final boolean isSim;
    double absEncoderHome = 0.14046377851159447;
    
    private Supplier<Double> autoAngleSupplier = null;

    SparkClosedLoopController closedLoopController;

    SparkMaxConfig turretConfig = new SparkMaxConfig();

    public TurretSubsystem() {
        isSim = Robot.isSimulation();

        closedLoopController = turretMotor.getClosedLoopController();

        turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.01)
                .i(0)
                .d(0)
                .outputRange(0, 135)
                .feedForward.kV(12.0 / 6784);

        turretConfig.closedLoop.positionWrappingEnabled(true);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        angleController.setTolerance(2.0);
        angleController.enableContinuousInput(-180, 180);

        //turretMotor.getEncoder().setPosition(((turretEncoder.get() - absEncoderHome)) * TurretConstants.TURRET_GEAR_RATIO);
        turretMotor.getEncoder().setPosition(0);
        SmartDashboard.putNumber("Turret P", 1);
        SmartDashboard.putNumber("Setpoint", 0);
    }

    @Override
    public void periodic() {
        goalAngleDeg = SmartDashboard.getNumber("Setpoint", 0);

        /*if (autoAngleSupplier != null) {
            Double suppliedAngle = autoAngleSupplier.get();
            if (suppliedAngle != null) {
                goalAngleDeg = normalizeAngle(suppliedAngle);
            }
        }*/
        
        double turretAngleDeg =
            isSim
                ? Math.toDegrees(turretSim.getTurretAngleRads())
                : Math.toDegrees(Math.toDegrees(getTurretAngleRads()));

        closedLoopController.setSetpoint((goalAngleDeg/360) * 135, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("Motor Setpoint", closedLoopController.getSetpoint());

        /*double mappedGoalDeg =
            chooseGoalAvoidingLimits(turretAngleDeg, goalAngleDeg);
        
        boolean takingLongPath =
            Math.abs(mappedGoalDeg - turretAngleDeg) > 180.0;

        if (takingLongPath) {
            angleController.setConstraints(
                new TrapezoidProfile.Constraints(500.0, 720.0)
            );
        } else {
            angleController.setConstraints(
                new TrapezoidProfile.Constraints(500.0, 720.0)
            );
        }*/

        double pidOut = angleController.calculate(turretAngleDeg, goalAngleDeg);

        double turretOut = Math.max(-1.0, Math.min(1.0, pidOut));

        if (turretAngleDeg <= 0 && pidOut < 0) turretOut = 0;
        if (turretAngleDeg >= 360 && pidOut > 0) turretOut = 0;

        if (angleController.atGoal()) {
            turretOut = 0;
        }

        //turretMotor.set(turretOut);


        SmartDashboard.putNumber("Turret Angle Measured (deg)", turretAngleDeg);
        SmartDashboard.putNumber("Turret Goal (deg)", goalAngleDeg);
        //SmartDashboard.putNumber("Turret Mapped Goal (deg)", mappedGoalDeg);
        SmartDashboard.putNumber(
            "Turret Setpoint (deg)",
            angleController.getSetpoint().position
        );
        SmartDashboard.putNumber(
            "Turret Vel Set",
            Math.toRadians(angleController.getSetpoint().velocity)
        );
        SmartDashboard.putNumber(
            "Turret Vel Read",
            turretSim.getTurretSpeed()
        );
        SmartDashboard.putNumber("Turret ABS Encoder Value", turretEncoder.get());

        SmartDashboard.putNumber("Turret Motor Encoder", turretMotor.getEncoder().getPosition());

        SmartDashboard.putNumber("Turret RAD", getTurretAngleRads());
        SmartDashboard.putNumber("Turret DEG", Math.toDegrees(getTurretAngleRads()));

        SmartDashboard.putNumber("Turret Goal Angle", goalAngleDeg);

        AlphaMechanism3d.setTurretAngle(
            isSim ? turretSim.getTurretAngleRads() : getTurretAngleRads()
        );

        if (RobotBase.isSimulation()) {
            turretSim.update(turretOut * 12.0, 0.02);
        }
    }

    public void setAngle(double angleDeg) {
        goalAngleDeg = angleDeg;
    }

    public boolean atGoal() {
        return angleController.atGoal();
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

    private static double placeGoalNearCurrent(double currentDeg, double targetDeg) {
        double delta = Math.IEEEremainder(targetDeg - currentDeg, 360.0);
        return currentDeg + delta;
    }
    
    private static double normalizeAngle(double angleDeg) {
        double normalized = angleDeg % 360.0;
        if (normalized < 0) {
            normalized += 360.0;
        }
        return normalized;
    }

    private static boolean pathCrossesLimit(double startDeg, double endDeg, double lowerLimit, double upperLimit) {
        double step = Math.signum(endDeg - startDeg);
        if (step == 0) return false;

        for (double a = startDeg; Math.abs(a - endDeg) > 0.5; a += step) {
            double norm = normalizeAngle(a);
            if (norm <= lowerLimit || norm >= upperLimit) {
                return true;
            }
        }
        return false;
    }

    private static double chooseGoalAvoidingLimits(double currentDeg, double targetDeg) {
        double shortGoal = placeGoalNearCurrent(currentDeg, targetDeg);

        // Long path = short ± 360
        double longGoal =
            shortGoal + (shortGoal > currentDeg ? -360.0 : 360.0);

        boolean shortHitsLimit =
            pathCrossesLimit(currentDeg, shortGoal, 0, 360);

        return shortHitsLimit ? longGoal : shortGoal;
    }

    public void setTurretSpeed(double speed) {
        turretMotor.set(speed);
    }
}
