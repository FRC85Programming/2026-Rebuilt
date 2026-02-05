package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ShotSolver;
import frc.robot.util.ShotSolver.ShotSolution;
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
    double goalAngle = 80;

    boolean isSim;

    public ShooterSubsystem() {
        isSim = Robot.isSimulation();
    }

    @Override
    public void periodic() {
        double flywheelRpmMeasured = isSim ? shooterSim.getFlywheelRPM() : getFlywheelRPM();
        double flywheelPIDOut = flywheelPID.calculate(flywheelRpmMeasured, goalRpm);
        double flywheelOut = flywheelPIDOut;
        flywheelOut = Math.max(-1.0, Math.min(1.0, flywheelOut));

        double hoodAngleMeasured = isSim ? shooterSim.getHoodAngleDeg() : getHoodAngle();
        double hoodPIDOut = hoodPID.calculate(hoodAngleMeasured, goalAngle);
        double hoodOut = hoodPIDOut;
        hoodOut = Math.max(-1.0, Math.min(1.0, hoodOut));

        flywheelMotor.set(flywheelOut);
        hoodMotor.set(hoodOut);

        if (isSim) {
            shooterSim.updateFlywheel(flywheelOut * 12.0, 0.02);
            shooterSim.updateHood(hoodOut, 0.02);
        }

        AlphaMechanism3d.setHoodAngle(getHoodAngle());

        SmartDashboard.putNumber("Flywheel Goal RPM", goalRpm);
        SmartDashboard.putNumber("Flywheel Measured RPM", flywheelRpmMeasured);
        SmartDashboard.putNumber("Sim Flywheel Speed", shooterSim.getFlywheelRPM());
        SmartDashboard.putNumber("Sim Hood Angle", shooterSim.getHoodAngleDeg());
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
        if (isSim) {
            return convertFlywheelVelocity(shooterSim.getFlywheelRPM());
        } else {
            return convertFlywheelVelocity(flywheelEncoder.getVelocity());
        }
    }

    public double getSimFlywheelRPM() {
        return convertFlywheelVelocity(shooterSim.getFlywheelRPM());
    }

    public double getHoodAngle() {
        if (isSim) {
            return getSimHoodAngle();
        } else {
            // Needs to be converted to degrees on real robot
            return hoodEncoder.getPosition();
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

    // TODO: These should be changed to come directly from the PID controllre
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

    public void calculateLookupTable() {
        // Calculate a placeholder table based on perfect physics
        for (var i=2; i <= 6.0; i+=0.5) {
            var solution = ShotSolver.solve(
                i,
                0.305,
                Constants.FieldConstants.blueHub.getZ(),
                Math.toRadians(65),
                Math.toRadians(Constants.ShooterConstants.HOOD_MIN_ANGLE),
                Math.toRadians(Constants.ShooterConstants.HOOD_MAX_ANGLE),
                0
            );
            ShotSolution shotSolution = solution.get();

            SmartDashboard.putNumber("TABLE Solved Speed" + i, mpsToRpm(shotSolution.launchVelocityMps()));
            SmartDashboard.putNumber("TABLE Solved Angle" + i, Math.toDegrees(shotSolution.launchAngleRad()));
        }
    }
}

