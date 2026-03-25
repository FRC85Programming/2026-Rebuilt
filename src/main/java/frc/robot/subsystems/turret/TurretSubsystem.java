package frc.robot.subsystems.turret;

import java.util.function.Supplier;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlphaMechanism3d;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.TimeOfFlightTable;

public class TurretSubsystem extends SubsystemBase {

    public enum TurretState {
        IDLE,
        AIMING,
        FEEDING,
        MANUALSHOOT,
        MANUALFEED
    }

    private final SparkFlex turretMotor =
        new SparkFlex(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

    private final TurretSim turretSim = new TurretSim();

    private final boolean isSim;

    SparkClosedLoopController closedLoopController;

    SparkMaxConfig turretConfig = new SparkMaxConfig();

    private double goalAngle = 0;

    private TurretState state = TurretState.IDLE;
    private SwerveSubsystem swerve = null;
    private Supplier<Translation3d> aimTarget = null;

    public TurretSubsystem() {
        isSim = Robot.isSimulation();

        closedLoopController = turretMotor.getClosedLoopController();

        turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.31)
                .i(0)
                .d(0)
                .outputRange(-0.6, 0.6)
                .positionWrappingEnabled(false)
                .feedForward.kV(12.0 / 6784);

        turretConfig.idleMode(IdleMode.kBrake);

        turretConfig.inverted(true);

        turretConfig.closedLoopRampRate(0.1);

        turretConfig.signals
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(500)
            .appliedOutputPeriodMs(100)
            .busVoltagePeriodMs(500)
            .outputCurrentPeriodMs(500)
            .motorTemperaturePeriodMs(1000)
            .faultsPeriodMs(500);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        turretMotor.getEncoder().setPosition(0);
        //turretMotor.getEncoder().setPosition((turretMotor.getAbsoluteEncoder().getPosition() - turretHome) * TurretConstants.TURRET_GEAR_RATIO);
        
        SmartDashboard.putNumber("Turret P", 1);
        SmartDashboard.putNumber("Setpoint", 0);

        SmartDashboard.putNumber("TOLERANCE", 9);
        SmartDashboard.getNumber("FEED TOLERANCE", 25);
    }

    @Override
    public void periodic() {
        if ((state == TurretState.AIMING || state == TurretState.FEEDING) && swerve != null && aimTarget != null) {
            updateAiming();
        } 

        updateLogging();

        if (RobotBase.isSimulation()) {
            AlphaMechanism3d.setTurretAngle(
                isSim ? turretSim.getTurretAngleRads() : getTurretAngleRads()
            );
        }
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

        double radialVel  =  robotVel.vxMetersPerSecond * toTargetDir.getX()
                           + robotVel.vyMetersPerSecond * toTargetDir.getY();
        double lateralVel = robotVel.vxMetersPerSecond * toTargetDir.getY()
                           + -robotVel.vyMetersPerSecond * toTargetDir.getX();

        // Iterate to find time of flight accounting for radial velocity compensation
        double effectiveDistance = distance;
        double timeOfFlight = 0;
        for (int i = 0; i < 3; i++) {
            timeOfFlight = TimeOfFlightTable.getTimeOfFlight(Math.max(effectiveDistance, 0.1));
            effectiveDistance = distance - (radialVel * timeOfFlight);
        }

        // Lead-compensated target position
        Translation2d leadTargetFieldPos = targetTranslation.toTranslation2d()
            .plus(new Translation2d(
                lateralVel * timeOfFlight,
                toTarget.getAngle().plus(Rotation2d.fromDegrees(90))
            ));

        Translation2d turretToTarget = leadTargetFieldPos.minus(turretFieldPos2d);
        Rotation2d fieldAngle  = new Rotation2d(turretToTarget.getX(), turretToTarget.getY());
        Rotation2d turretAngle = fieldAngle.minus(robotPose.getRotation());

        setTurretAngle(turretAngle.getDegrees() - TurretConstants.MOUNTING_OFFSET);

        SmartDashboard.putNumber("Calced Turret Angle", turretAngle.getDegrees());
    }

    /** Enter AIMING state: periodic will automatically track the target each loop. */
    public void startAiming(SwerveSubsystem swerve, Supplier<Translation3d> target) {
        this.swerve = swerve;
        this.aimTarget = target;
        this.state = TurretState.AIMING;
    }

    /** Enter FEEDING state: tracks the feed target using the same aiming logic. */
    public void startFeeding(SwerveSubsystem swerve, Supplier<Translation3d> target) {
        this.swerve = swerve;
        this.aimTarget = target;
        this.state = TurretState.FEEDING;
    }

    public void startManualFeeding(SwerveSubsystem swerve) {
        this.swerve = swerve;
        setTurretAngle(90);
        this.state = TurretState.MANUALFEED;
    }

    public void startManualShooting(SwerveSubsystem swerve) {
        this.swerve = swerve;
        setTurretAngle(0);
        this.state = TurretState.MANUALSHOOT;
    }

    /** Return to IDLE state and stop auto-tracking. */
    public void stopAiming() {
        this.state = TurretState.IDLE;
        this.aimTarget = null;
        this.swerve = null;
        setTurretAngle(0);
    }

    public TurretState getState() {
        return state;
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
        return (turretMotor.getEncoder().getPosition() / TurretConstants.TURRET_GEAR_RATIO) * (Math.PI * 2);
    }

    public void setTurretSpeed(double speed) {
        turretMotor.set(speed);
    }

    public void updateLogging() {
        double motorPosition = turretMotor.getEncoder().getPosition();
        double turretAngleRads = getTurretAngleRads();
        double turretAngleDeg = Math.toDegrees(turretAngleRads);

        SmartDashboard.putNumber("Motor Setpoint", closedLoopController.getSetpoint());
        SmartDashboard.putNumber("Turret Angle Measured (deg)", turretAngleDeg);
        SmartDashboard.putNumber("Turret ABS Encoder Value", turretMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Turret Motor Encoder", motorPosition);
        SmartDashboard.putNumber("Turret RAD", turretAngleRads);
        SmartDashboard.putNumber("Turret DEG", turretAngleDeg);
        SmartDashboard.putString("Turret State", state.toString());
    }

    public boolean turretAtAngle(double tolerance) {
        double currentDeg = Math.toDegrees(getTurretAngleRads());
        double goalDeg = (goalAngle / TurretConstants.TURRET_GEAR_RATIO) * 360.0;
        return Math.abs(currentDeg - goalDeg) < tolerance;
    }

    public void manualFeedPosition() {
        setTurretAngle(90);
    }

    public void manualShootPosition() {
        setTurretAngle(0);
    }
}
