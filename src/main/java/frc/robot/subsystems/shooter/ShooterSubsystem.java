package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

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

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.BallisticTrajectory3d;
import frc.robot.util.FeedingTable;
import frc.robot.util.ShooterTable;
import frc.robot.util.TimeOfFlightTable;
import frc.robot.util.TrajectoryTransform3d;

public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterState {
        IDLE,
        AIMING,
        FEEDING
    }

    private final SparkFlex flywheelMotorLeft =
        new SparkFlex(14, MotorType.kBrushless);

    private final SparkFlex flywheelMotorRight =
        new SparkFlex(17, MotorType.kBrushless);

    private final SparkFlex hoodMotor =
        new SparkFlex(ShooterConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

    private final ShooterSim shooterSim = new ShooterSim();

    double goalRpm = 0.0;
    double goalAngle = 69;

    boolean readyToFire = false;

    boolean isSim;

    double hoodHome = 0.6361150741577148;

    SparkClosedLoopController leftController;
    SparkClosedLoopController rightController;

    SparkFlexConfig flywheelConfigRight = new SparkFlexConfig();
    SparkFlexConfig flywheelConfigLeft = new SparkFlexConfig();
    SparkFlexConfig hoodConfig = new SparkFlexConfig();

    private ShooterState state = ShooterState.IDLE;
    private SwerveSubsystem swerve = null;
    private Supplier<Translation3d> aimTarget = null;
    private double calculatedRPM = 0.0;

    public ShooterSubsystem() {
        isSim = Robot.isSimulation();

        leftController = flywheelMotorLeft.getClosedLoopController();
        rightController = flywheelMotorRight.getClosedLoopController();

        flywheelConfigRight.closedLoop
               .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.00000005)
                .i(0)
                .d(0)
                .outputRange(-1, 0)
                .p(0.0004, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 0, ClosedLoopSlot.kSlot1)
                .feedForward
                .kV(12.0 / 6784, ClosedLoopSlot.kSlot1);

        flywheelConfigLeft.closedLoop
               .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.00000005)
                .i(0)
                .d(0)
                .outputRange(0, 1)
                .p(0.0004, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(0, 1, ClosedLoopSlot.kSlot1)
                .feedForward
                .kV(12.0 / 6784, ClosedLoopSlot.kSlot1);

        hoodConfig.closedLoop
               .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.5)
                .i(0)
                .d(0)
                .outputRange(-0.5, 0.5)
                .feedForward.kV(12.0 / 6784);

        flywheelConfigRight.idleMode(IdleMode.kCoast);
        flywheelConfigLeft.idleMode(IdleMode.kCoast);

        flywheelConfigLeft.signals
            .primaryEncoderVelocityPeriodMs(20)
            .primaryEncoderPositionPeriodMs(500)
            .appliedOutputPeriodMs(100)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        flywheelConfigRight.signals
            .primaryEncoderVelocityPeriodMs(20)
            .primaryEncoderPositionPeriodMs(500)
            .appliedOutputPeriodMs(100)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        hoodConfig.inverted(true);

        hoodConfig.signals
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(500)
            .absoluteEncoderPositionPeriodMs(500)
            .absoluteEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(100)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        flywheelMotorLeft.configure(flywheelConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotorRight.configure(flywheelConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("TUNE Shot RPM", 0);
        SmartDashboard.putNumber("TUNE Shot Angle", 75);
        SmartDashboard.putNumber("Set Hood Angle", 69);

        hoodMotor.getEncoder().setPosition((hoodMotor.getAbsoluteEncoder().getPosition() - hoodHome) * 10.96);
    }

    @Override
    public void periodic() {
        if ((state == ShooterState.AIMING || state == ShooterState.FEEDING) && swerve != null && aimTarget != null) {
            updateAiming();
        }

        if (isSim) {
            AlphaMechanism3d.setHoodAngle(getHoodAngle());
        }

        double encoderPos = hoodMotor.getEncoder().getPosition();
        double hoodAngle = (((encoderPos * 360) / 9) / 10.96) + 75;

        SmartDashboard.putNumber("Flywheel Goal RPM", goalRpm);
        SmartDashboard.putNumber("Flywheel Measured RPM", getFlywheelRPM());
        SmartDashboard.putNumber("Sim Flywheel Speed", shooterSim.getFlywheelRPM());
        SmartDashboard.putNumber("Sim Hood Angle", shooterSim.getHoodAngleDeg());
        SmartDashboard.putNumber("Hood Encoder ABS", hoodMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Hood Encoder", encoderPos * 360);
        SmartDashboard.putNumber("Hood Encoder Converted", encoderPos * 360 / 9);
        SmartDashboard.putNumber("Hood Angle", hoodAngle);
        SmartDashboard.putNumber("Hood Goal Angle", goalAngle);
        SmartDashboard.putString("Shooter State", state.toString());
    }

    private void updateAiming() {
        Translation3d targetTranslation = aimTarget.get();
        ChassisSpeeds robotVel = swerve.getFieldVelocity();
        Pose2d robotPose = swerve.getPose();
        ChassisSpeeds robotRelVel = swerve.getRobotVelocity();

        final double PHASE_DELAY = 0.03;
        robotPose = robotPose.exp(new Twist2d(
            robotRelVel.vxMetersPerSecond * PHASE_DELAY,
            robotRelVel.vyMetersPerSecond * PHASE_DELAY,
            robotRelVel.omegaRadiansPerSecond * PHASE_DELAY
        ));

        Translation2d turretFieldPos2d =
            robotPose.getTranslation()
                .plus(TurretConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d().rotateBy(robotPose.getRotation()));

        Translation2d toTarget = targetTranslation.toTranslation2d().minus(turretFieldPos2d);
        double distance = toTarget.getNorm();

        if (distance < 1e-6) return;

        Translation2d toTargetDir = toTarget.div(distance);

        // Radial velocity: positive = moving toward target (reduces effective distance)
        double radialVel =  robotVel.vxMetersPerSecond * toTargetDir.getX()
                          + robotVel.vyMetersPerSecond * toTargetDir.getY();

        // Iterate to find effective distance accounting for radial velocity
        double effectiveDistance = distance;
        double timeOfFlight = 0;
        for (int i = 0; i < 3; i++) {
            timeOfFlight = TimeOfFlightTable.getTimeOfFlight(Math.max(effectiveDistance, 0.1));
            effectiveDistance = distance - (-radialVel * timeOfFlight);
        }

        SmartDashboard.putNumber("CALCULATED TIME OF FLIGHT", timeOfFlight);

        double clampedEffectiveDistance = Math.max(effectiveDistance, 0.1);
        var setpoint = (state == ShooterState.FEEDING)
            // CHANGE THIS TO FEEDING TABLE ONCE IT IS CREATED
            ? ShooterTable.getSetpoint(clampedEffectiveDistance)
            : ShooterTable.getSetpoint(clampedEffectiveDistance);

        calculatedRPM = Math.abs(setpoint.flywheelRPM());
        double hoodAngleRad = setpoint.hoodAngle().getRadians();

        setHoodAngle(Math.toDegrees(hoodAngleRad));

        // Trajectory visualization
        Pose3d[] poses = TrajectoryTransform3d.toFieldRelative(
            robotPose.getTranslation(),
            robotPose.getRotation(),
            BallisticTrajectory3d.generate(rpmToMps(calculatedRPM), hoodAngleRad, 0.305, 3.0, 0.02)
        ).stream()
            .map(p -> new Pose3d(p, new Rotation3d()))
            .toArray(Pose3d[]::new);

        Logger.recordOutput("Shot/Trajectory3d", poses);
        SmartDashboard.putNumber("Calculated Shot Angle (deg)", Math.toDegrees(hoodAngleRad));
        SmartDashboard.putNumber("Calculated Shot Speed (rpm)", calculatedRPM);
    }

    /** Enter AIMING state: periodic will automatically compute and track the hood angle each loop. */
    public void startAiming(SwerveSubsystem swerve, Supplier<Translation3d> target) {
        this.swerve = swerve;
        this.aimTarget = target;
        this.state = ShooterState.AIMING;
    }

    /** Enter FEEDING state: same as AIMING but uses FeedingTable for setpoints. */
    public void startFeeding(SwerveSubsystem swerve, Supplier<Translation3d> target) {
        this.swerve = swerve;
        this.aimTarget = target;
        this.state = ShooterState.FEEDING;
    }

    /** Return to IDLE state and stop auto-tracking. Clears trajectory visualization. */
    public void stopAiming() {
        this.state = ShooterState.IDLE;
        this.aimTarget = null;
        this.swerve = null;
        this.calculatedRPM = 0.0;
        setHoodAngle(ShooterConstants.HOOD_HOME_ANGLE);
        Logger.recordOutput("Shot/Trajectory3d", new Pose3d[0]);
    }

    public ShooterState getState() {
        return state;
    }

    /**
     * Returns the flywheel RPM calculated during the last AIMING periodic update.
     * Use this in the Shoot command to spin up the flywheel to the correct speed.
     */
    public double getCalculatedRPM() {
        return calculatedRPM;
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
            // Using left motor results in a positive value
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
            return (((hoodMotor.getEncoder().getPosition() * 360) / 9) / 10.96) + 75;
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
        return getFlywheelRPM() / goalRpm > tolerance;
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

}
