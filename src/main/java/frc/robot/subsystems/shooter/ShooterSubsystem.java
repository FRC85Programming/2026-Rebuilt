package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

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

    private final SparkFlex beltMotor =
        new SparkFlex(ShooterConstants.BELT_MOTOR_ID, MotorType.kBrushless);

    private final ShooterSim shooterSim = new ShooterSim();


    double goalRpm = 0.0;
    double goalAngle = 69;

    boolean readyToFire = false;

    boolean isSim;

    double hoodHome = 0.830276608467102;

    SparkClosedLoopController leftController;

    SparkClosedLoopController rightController;

    SparkFlexConfig flywheelConfigRight = new SparkFlexConfig();
    SparkFlexConfig flywheelConfigLeft = new SparkFlexConfig();
    SparkFlexConfig hoodConfig = new SparkFlexConfig();

    public ShooterSubsystem() {
        isSim = Robot.isSimulation();

        leftController = flywheelMotorLeft.getClosedLoopController();
        rightController = flywheelMotorRight.getClosedLoopController();

        flywheelConfigRight.closedLoop
               .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.00000005)
                .i(0)
                .d(0)
                .outputRange(-1, 0)
                // Set PID values for velocity control in slot 1
                .p(0.0004, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 0, ClosedLoopSlot.kSlot1)
                .feedForward
                // kV is now in Volts, so we multiply by the nominal voltage (12V)
                .kV(12.0 / 6784, ClosedLoopSlot.kSlot1);

        flywheelConfigLeft.closedLoop
               .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.00000005)
                .i(0)
                .d(0)
                .outputRange(0, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0004, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(0, 1, ClosedLoopSlot.kSlot1)
                .feedForward
                // kV is now in Volts, so we multiply by the nominal voltage (12V)
                .kV(12.0 / 6784, ClosedLoopSlot.kSlot1);

        hoodConfig.closedLoop
               .feedbackSensor(FeedbackSensor.kDetachedAbsoluteEncoder)
                .p(0.35)
                .i(0)
                .d(0)
                .outputRange(-0.5, 0.5)
                .feedForward.kV(12.0 / 6784);
        
        flywheelConfigRight.idleMode(IdleMode.kCoast);
        flywheelConfigLeft.idleMode(IdleMode.kCoast);

        hoodConfig.inverted(true);

        flywheelMotorLeft.configure(flywheelConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotorRight.configure(flywheelConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("TUNE Shot RPM", 0);
        SmartDashboard.putNumber("TUNE Shot Angle", 75);
        SmartDashboard.putNumber("Set Hood Angle", 69);

        hoodMotor.getEncoder().setPosition((hoodMotor.getAbsoluteEncoder().getPosition() - hoodHome)*10.96);
    }

    @Override
    public void periodic() {
        if (isSim) {
            //shooterSim.updateFlywheel(flywheelOut * 12.0, 0.02);
            //shooterSim.updateHood(hoodOut * 12.0, 0.02);
            AlphaMechanism3d.setHoodAngle(getHoodAngle());
        }

        SmartDashboard.putNumber("Flywheel Goal RPM", goalRpm);
        SmartDashboard.putNumber("Flywheel Measured RPM", getFlywheelRPM());
        SmartDashboard.putNumber("Sim Flywheel Speed", shooterSim.getFlywheelRPM());
        SmartDashboard.putNumber("Sim Hood Angle", shooterSim.getHoodAngleDeg());
        SmartDashboard.putNumber("Hood Encoder ABS", hoodMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Hood Encoder", hoodMotor.getEncoder().getPosition()*360);
        SmartDashboard.putNumber("Hood Encoder Converted", hoodMotor.getEncoder().getPosition()*360/9);
        SmartDashboard.putNumber("Hood Angle", (((hoodMotor.getEncoder().getPosition()*360) /9) / 10.96) + 75);
        SmartDashboard.putNumber("Hood Goal Angle", goalAngle);
    }

    private double convertFlywheelVelocity(double velocity) {
        return velocity / ShooterConstants.FLYWHEEL_GEAR_RATIO;
    }

    public void setFlywheelRPM(double rpm) {
        goalRpm = rpm;
        leftController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        rightController.setSetpoint(-rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    public void stopFlywheel() {
        goalRpm = 0;
        flywheelMotorLeft.set(0);
        flywheelMotorRight.set(0);
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
            return (((hoodMotor.getEncoder().getPosition()*360) /9) / 10.96) + 75;
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
        hoodMotor.getClosedLoopController().setSetpoint(((angle - 75) / 360) * 9 * 10.96, ControlType.kPosition, ClosedLoopSlot.kSlot0);    
    }

    public boolean hoodAtAngle(double tolerance) {
        return Math.abs(getHoodAngle() - goalAngle) < tolerance;
    }

    public boolean flywheelAtSpeed(double tolerance) {
        return getFlywheelRPM()/goalRpm > tolerance;
    }

    public double mpsToRpm(double speed) {
        return speed / 0.0023;
    }

    public double rpmToMps(double rpm) {
        return rpm * 0.0023;
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
        }
    }

    public void setFeedSpeed(double speed) {
        feedMotor.set(speed);
        beltMotor.set(-speed);
    }
}

