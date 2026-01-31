package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
    private final SparkFlex turretMotor =
        new SparkFlex(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

    private final AbsoluteEncoder turretEncoder = turretMotor.getAbsoluteEncoder();

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
    
    private Supplier<Double> autoAngleSupplier = null;
    
    private static final double LOWER_LIMIT = 5.0;
    private static final double UPPER_LIMIT = 355.0;

    public TurretSubsystem() {
        isSim = Robot.isSimulation();

        angleController.setTolerance(1.0);
    }

    @Override
    public void periodic() {
        if (autoAngleSupplier != null) {
            Double suppliedAngle = autoAngleSupplier.get();
            if (suppliedAngle != null) {
                goalAngleDeg = getSmartWrappedGoal(
                    isSim ? Math.toDegrees(turretSim.getTurretAngleRads()) : Math.toDegrees(getTurretAngleRads()),
                    suppliedAngle
                );
            }
        }
        
        double turretAngleDeg =
            isSim
                ? Math.toDegrees(turretSim.getTurretAngleRads())
                : Math.toDegrees(getTurretAngleRads());

        double mappedGoalDeg =
            placeGoalNearCurrent(turretAngleDeg, goalAngleDeg);

        double pidOut = angleController.calculate(turretAngleDeg, mappedGoalDeg);

        double turretOut = Math.max(-1.0, Math.min(1.0, pidOut));

        if (turretAngleDeg <= 0 && pidOut < 0) turretOut = 0;
        if (turretAngleDeg >= 360 && pidOut > 0) turretOut = 0;

        if (angleController.atGoal()) {
            turretOut = 0;
        }

        turretMotor.set(turretOut);

        SmartDashboard.putNumber("Turret Angle Measured (deg)", turretAngleDeg);
        SmartDashboard.putNumber("Turret Goal (deg)", goalAngleDeg);
        SmartDashboard.putNumber("Turret Mapped Goal (deg)", mappedGoalDeg);
        SmartDashboard.putNumber(
            "Turret Setpoint (deg)",
            angleController.getSetpoint().position
        );

        AlphaMechanism3d.setTurretAngle(
            isSim ? turretSim.getTurretAngleRads() : getTurretAngleRads()
        );

        if (RobotBase.isSimulation()) {
            turretSim.update(turretOut * 12.0, 0.02);
        }
    }

    /**
     * Sets the turret's goal angle (degrees)
     */
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
        return turretEncoder.getPosition();
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
    
    // This doesn't need to be a setpoint switch, just a power switch
    private double getSmartWrappedGoal(double currentAngle, double desiredAngle) {
        double normalizedDesired = normalizeAngle(desiredAngle);
        double normalizedCurrent = normalizeAngle(currentAngle);
        
        double directPath = placeGoalNearCurrent(normalizedCurrent, normalizedDesired);
        
        if (directPath > UPPER_LIMIT && normalizedCurrent < LOWER_LIMIT) {
            return normalizedDesired - 360.0;
        }
        
        if (directPath < LOWER_LIMIT && normalizedCurrent > UPPER_LIMIT) {
            return normalizedDesired + 360.0;
        }
        
        if (normalizedCurrent > UPPER_LIMIT && normalizedDesired < LOWER_LIMIT) {
            double goingForward = 360.0 - normalizedCurrent + normalizedDesired;
            double goingBackward = normalizedCurrent - normalizedDesired;
            if (goingBackward < goingForward) {
                return normalizedDesired;
            }
        }
        
        if (normalizedCurrent < LOWER_LIMIT && normalizedDesired > UPPER_LIMIT) {
            double goingBackward = normalizedCurrent + (360.0 - normalizedDesired);
            double goingForward = normalizedDesired - normalizedCurrent;
            if (goingForward < goingBackward) {
                return normalizedDesired;
            }
        }
        
        return normalizedDesired;
    }
    
    private static double normalizeAngle(double angleDeg) {
        double normalized = angleDeg % 360.0;
        if (normalized < 0) {
            normalized += 360.0;
        }
        return normalized;
    }
}
