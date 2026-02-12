package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ShooterTable;
import frc.robot.util.ShotSolver;
import frc.robot.util.ShotSolver.ShotSolution;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShooterSubsystem extends SubsystemBase{
     private final SparkFlex flywheelMotorLeft =
      new SparkFlex(14, MotorType.kBrushless);

    private final SparkFlex flywheelMotorRight =
      new SparkFlex(17, MotorType.kBrushless);

    private final SparkFlex hoodMotor =
        new SparkFlex(ShooterConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlex feedMotor =
        new SparkFlex(ShooterConstants.FEED_MOTOR_ID, MotorType.kBrushless);

    private final ShooterSim shooterSim = new ShooterSim();

    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(0);

    private final PIDController flywheelPID = new PIDController(0.00007, 0.000166, 0.000004);
    private final PIDController hoodPID = new PIDController(0.035, 0, 0);

    double goalRpm = 0.0;
    double goalAngle = 69;

    boolean readyToFire = false;

    boolean isSim;

    double hoodHome = 0.890926322273158;

    public ShooterSubsystem() {
        isSim = Robot.isSimulation();

        SmartDashboard.putNumber("Flywheel P Value", 0.00007);
        SmartDashboard.putNumber("Flywheel I Value", 0.000166);
        SmartDashboard.putNumber("Flywheel D Value", 0.000004);

        SmartDashboard.putNumber("TUNE Shot RPM", 0);
        SmartDashboard.putNumber("TUNE Shot Angle", 80);
        SmartDashboard.putNumber("Set Hood Angle", 69);
        hoodMotor.getEncoder().setPosition((hoodEncoder.get() - hoodHome)*9);
    }

    @Override
    public void periodic() {
        flywheelPID.setP(SmartDashboard.getNumber("Flywheel P Value", 0.00007));
        flywheelPID.setI(SmartDashboard.getNumber("Flywheel I Value", 0.000166));
        flywheelPID.setD(SmartDashboard.getNumber("Flywheel D Value", 0.000004));


        final double flywheelRpmMeasured = getFlywheelRPM();
        final double flywheelPIDOut = flywheelPID.calculate(flywheelRpmMeasured, goalRpm);
        double flywheelOut = flywheelPIDOut;
        flywheelOut = Math.max(-1.0, Math.min(1.0, flywheelOut));

        if (goalRpm != 0) {
            setFlywheelSpeed(flywheelOut);
        } else {
            setFlywheelSpeed(0);
        }

        final double hoodAngle = getHoodAngle();
        final double hoodPIDOut = hoodPID.calculate(hoodAngle, goalAngle);
        double hoodOut = hoodPIDOut;
        hoodOut = Math.max(-1.0, Math.min(1.0, hoodOut));

        hoodMotor.set(hoodOut);

        if (isSim) {
            shooterSim.updateFlywheel(flywheelOut * 12.0, 0.02);
            shooterSim.updateHood(hoodOut * 12.0, 0.02);
        }

        AlphaMechanism3d.setHoodAngle(getHoodAngle());

        SmartDashboard.putNumber("Flywheel Goal RPM", goalRpm);
        SmartDashboard.putNumber("Flywheel Measured RPM", flywheelRpmMeasured);
        SmartDashboard.putNumber("Sim Flywheel Speed", shooterSim.getFlywheelRPM());
        SmartDashboard.putNumber("Sim Hood Angle", shooterSim.getHoodAngleDeg());
        SmartDashboard.putNumber("Flywheel PID Out", flywheelPIDOut);
        SmartDashboard.putNumber("Flywheel Out", flywheelOut);
        SmartDashboard.putNumber("Hood Encoder ABS", hoodEncoder.get());
        SmartDashboard.putNumber("Hood Encoder", hoodMotor.getEncoder().getPosition()*360);
        SmartDashboard.putNumber("Hood Encoder Converted", hoodMotor.getEncoder().getPosition()*360/9);
        SmartDashboard.putNumber("Hood Angle", (((hoodMotor.getEncoder().getPosition()*360) /9) / 10.6) + 80);
        
        SmartDashboard.putNumber("Hood Goal Angle", goalAngle);
        SmartDashboard.putNumber("Hood Measured Angle", hoodAngleMeasured);
        SmartDashboard.putNumber("Hood PID Out", hoodPIDOut);
        SmartDashboard.putNumber("Hood Out", hoodOut);
        
        Logger.recordOutput("Shooter/FlywheelGoalRPM", goalRpm);
        Logger.recordOutput("Shooter/FlywheelMeasuredRPM", flywheelRpmMeasured);
        Logger.recordOutput("Shooter/FlywheelOut", flywheelOut);
        Logger.recordOutput("Shooter/HoodGoalAngle", goalAngle);
        Logger.recordOutput("Shooter/HoodMeasuredAngle", hoodAngleMeasured);
        Logger.recordOutput("Shooter/HoodOut", hoodOut);
    }

    private double convertFlywheelVelocity(double velocity) {
        return velocity / ShooterConstants.FLYWHEEL_GEAR_RATIO;
    }

    public void setFlywheelRPM(double rpm) {
        goalRpm = rpm;
    }

    public void stopFlywheel() {
        flywheelMotorLeft.stopMotor();
        flywheelMotorRight.stopMotor();
    }

    public double getFlywheelRPM() {
        if (isSim) {
            return convertFlywheelVelocity(shooterSim.getFlywheelRPM());
        } else {
            return convertFlywheelVelocity(flywheelMotorLeft.getEncoder().getVelocity());
        }
    }

    public double getSimFlywheelRPM() {
        return convertFlywheelVelocity(shooterSim.getFlywheelRPM());
    }

    public double getHoodAngle() {
        if (isSim) {
            return getSimHoodAngle();
        } else {
            // Assume encoder is in rotations?
            return (((hoodMotor.getEncoder().getPosition()*360) /9) / 10.6) + 80;
        }
    }

    public double getSimHoodAngle() {
        return shooterSim.getHoodAngleDeg();
    }

    // In degrees
    public void setHoodAngle(double angle) {
        angle = Math.max(
            ShooterConstants.HOOD_MIN_ANGLE,
            Math.min(angle, ShooterConstants.HOOD_MAX_ANGLE));
           
        goalAngle = angle;
    }

    // TODO: These should be changed to come directly from the PID controller
    public boolean hoodAtAngle(double tolerance) {
        return Math.abs(getHoodAngle() - goalAngle) < tolerance;
    }

    public boolean flywheelAtSpeed(double tolerance) {
        return Math.abs(getFlywheelRPM() - goalRpm) < tolerance;
    }

    public double mpsToRpm(double speed) {
        // Assume 16m/s = 6000 RPM as a placeholder
        return speed * 375.0;
    }

    public double rpmToMps(double rpm) {
        return rpm / 375.0;
    }

    public void simulatedShot(Pose2d pose, ChassisSpeeds velocity, Rotation2d turretAngle) {
        shooterSim.generateProjectile(pose, velocity, turretAngle);
    }

    public boolean generateProjectileIsReady() {
        return shooterSim.generateProjectileIsReady();
    }

    public void cleanupPieces() {
        shooterSim.cleanupPieces();
    }

    public void setFlywheelSpeed(double speed) {
        flywheelMotorLeft.set(speed);
        flywheelMotorRight.set(-speed);
    }
    public void calculateLookupTable() {
        // Calculate a placeholder table based on perfect physics
        for (var i=2; i <= 6.0; i+=0.5) {
            var solution = ShotSolver.solve(
                i,
                0.305,
                FieldConstants.blueHub.getZ(),
                Math.toRadians(65),
                Math.toRadians(Constants.ShooterConstants.HOOD_MIN_ANGLE),
                Math.toRadians(Constants.ShooterConstants.HOOD_MAX_ANGLE),
                0
            );
            ShotSolution shotSolution = solution.get();

    public void setFeedSpeed(double speed) {
        feedMotor.set(speed);
    }
}

