package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
     private final SparkFlex flywheelMotor =
      new SparkFlex(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex hoodMotor =
        new SparkFlex(ShooterConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

    private final ShooterSim shooterSim = new ShooterSim();

    private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

    private final PIDController flywheelPID = new PIDController(ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D);
    private final PIDController hoodPID = new PIDController(ShooterConstants.HOOD_P, ShooterConstants.HOOD_I, ShooterConstants.HOOD_D);

    double goalRpm = 0.0;

    @Override
    public void periodic() {
        final boolean isSim = Robot.isSimulation();
        final double flywheelRpmMeasured = isSim ? shooterSim.getFlywheelRPM() : getFlywheelRPM();

        // Simple feedforward in "percent output per RPM" (roughly 1 / freeSpeedRPM).
        final double flywheelFF = ShooterConstants.FLYWHEEL_FF * goalRpm;
        final double flywheelPIDOut = flywheelPID.calculate(flywheelRpmMeasured, goalRpm);
        double flywheelOut = flywheelFF + flywheelPIDOut;
        flywheelOut = Math.max(-1.0, Math.min(1.0, flywheelOut));

        flywheelMotor.set(flywheelOut);

        if (isSim) {
            shooterSim.update(flywheelOut * 12.0, 0.02);
        }

        SmartDashboard.putNumber("Flywheel Goal RPM", goalRpm);
        SmartDashboard.putNumber("Flywheel Measured RPM", flywheelRpmMeasured);
        SmartDashboard.putNumber("Sim Flywheel Speed", shooterSim.getFlywheelRPM());
        SmartDashboard.putNumber("Sim Hood Angle", shooterSim.getHoodAngleDeg());
        SmartDashboard.putNumber("Flywheel FF Out", flywheelFF);
        SmartDashboard.putNumber("Flywheel PID Out", flywheelPIDOut);
        SmartDashboard.putNumber("Flywheel Out", flywheelOut);
    }

    private double convertFlywheelVelocity(double velocity) {
        return velocity / ShooterConstants.FLYWHEEL_GEAR_RATIO;
    }

    public void setFlywheelRPM(double rpm) {
        goalRpm = rpm;
    }

    public void stopFlywheel() {
        flywheelMotor.stopMotor();
    }

    public double getFlywheelRPM() {
        return convertFlywheelVelocity(flywheelEncoder.getVelocity());
    }

    public double getSimFlywheelRPM() {
        return convertFlywheelVelocity(shooterSim.getFlywheelRPM());
    }

    public double getHoodAngle() {
        return hoodEncoder.getPosition();
    }

    public double getSimHoodAngle() {
        return shooterSim.getHoodAngleDeg();
    }

    // In degrees
    public void setHoodAngle(double angle) {
        angle = Math.max(
            ShooterConstants.HOOD_MIN_ANGLE,
            Math.min(angle, ShooterConstants.HOOD_MAX_ANGLE));
           
        if (Robot.isSimulation()) {    
            shooterSim.setHoodAngle(angle);
        }
        hoodMotor.set(hoodPID.calculate(getHoodAngle(), angle));
    }

    public boolean hoodAtAngle(double targetDeg, double tolerance) {
        return Math.abs(getHoodAngle() - targetDeg) < tolerance;
    }

    public boolean flywheelAtSpeed(double targetRPM, double tolerance) {
        return Math.abs(getFlywheelRPM() - targetRPM) < tolerance;
    }
}

