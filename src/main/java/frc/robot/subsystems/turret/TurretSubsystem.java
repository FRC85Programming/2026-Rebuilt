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

    public TurretSubsystem() {
        isSim = Robot.isSimulation();

        angleController.setTolerance(1.0);
    }

    @Override
    public void periodic() {
        if (autoAngleSupplier != null) {
            Double suppliedAngle = autoAngleSupplier.get();
            if (suppliedAngle != null) {
                goalAngleDeg = suppliedAngle;
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

        if (turretAngleDeg <= 0 && pidOut < 0) pidOut = 0;
        if (turretAngleDeg >= 360 && pidOut > 0) pidOut = 0;

        if (angleController.atGoal()) {
            turretOut = 0;
        }

        turretMotor.set(turretOut);

        SmartDashboard.putNumber("Turret Angle Measured (deg)", turretAngleDeg);
        SmartDashboard.putNumber("Turret Goal (deg)", goalAngleDeg);
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
}
