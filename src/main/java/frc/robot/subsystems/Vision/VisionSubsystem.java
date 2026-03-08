package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class VisionSubsystem extends SubsystemBase {

    private static final String DETECTION_CAMERA_NAME = "Balls1";

    private static final Transform3d ROBOT_TO_DETECTION_CAM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(13.75),
                0.0,
                Units.inchesToMeters(19.5)
            ),
            new Rotation3d(0.0, Units.degreesToRadians(15.0), 0.0)
        );

    private static final double DETECTION_CAM_HEIGHT_METERS = Units.inchesToMeters(19.5);
    private static final double DETECTION_CAM_PITCH_DEGREES = 15.0;
    private static final double BALL_HEIGHT_METERS = 0.075;
    private static final int FUEL_CLASS_ID = 0;
    private static final double MIN_DETECTION_CONFIDENCE = 0.3;

    private final PhotonCamera detectionCamera;
    private Supplier<Pose2d> poseSupplier = null;

    public VisionSubsystem() {
        detectionCamera = new PhotonCamera(DETECTION_CAMERA_NAME);
    }

    public void setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier;
    }

    @Override
    public void periodic() {
        Optional<Double> distance = getBallDistanceMeters();
        SmartDashboard.putBoolean("Vision/CameraConnected", detectionCamera.isConnected());
        SmartDashboard.putBoolean("Vision/BallDetected", distance.isPresent());
        SmartDashboard.putNumber("Vision/BallDistanceMeters", distance.orElse(-1.0));

        if (poseSupplier != null) {
            List<Translation3d> balls = getAllBallFieldTranslations();
            SmartDashboard.putNumber("Vision/BallCount", balls.size());
            Logger.recordOutput("Vision/BallPosition3d", balls.toArray(new Translation3d[0]));
        }
    }

    /** Returns the distance to the closest detected ball. */
    public Optional<Double> getBallDistanceMeters() {
        var result = detectionCamera.getLatestResult();
        if (!result.hasTargets()) return Optional.empty();

        return result.getTargets().stream()
            .filter(t -> t.getDetectedObjectClassID() == FUEL_CLASS_ID)
            .filter(t -> t.getDetectedObjectConfidence() >= MIN_DETECTION_CONFIDENCE)
            .findFirst()
            .map(t -> Math.abs(PhotonUtils.calculateDistanceToTargetMeters(
                DETECTION_CAM_HEIGHT_METERS,
                BALL_HEIGHT_METERS,
                Units.degreesToRadians(DETECTION_CAM_PITCH_DEGREES),
                Units.degreesToRadians(-t.getPitch())
            )));
    }

    /**
     * Returns field-relative Translation3d positions for ALL detected balls.
     * No median filter is applied — tracking multiple balls across frames
     * requires object association logic beyond a simple deque history.
     */
    public List<Translation3d> getAllBallFieldTranslations() {
        List<Translation3d> ballPositions = new ArrayList<>();

        if (poseSupplier == null) return ballPositions;

        var result = detectionCamera.getLatestResult();
        if (!result.hasTargets()) return ballPositions;

        var targets = result.getTargets().stream()
            .filter(t -> t.getDetectedObjectClassID() == FUEL_CLASS_ID)
            .filter(t -> t.getDetectedObjectConfidence() >= MIN_DETECTION_CONFIDENCE)
            .collect(Collectors.toList());

        if (targets.isEmpty()) return ballPositions;

        Pose2d robotPose = poseSupplier.get();
        double heading = robotPose.getRotation().getRadians();

        for (var t : targets) {
            double distance = Math.abs(PhotonUtils.calculateDistanceToTargetMeters(
                DETECTION_CAM_HEIGHT_METERS,
                BALL_HEIGHT_METERS,
                Units.degreesToRadians(DETECTION_CAM_PITCH_DEGREES),
                Units.degreesToRadians(-t.getPitch())
            ));
            double yawRad = Units.degreesToRadians(-t.getYaw());

            // Camera-frame position
            Translation3d ballInCamFrame = new Translation3d(
                distance * Math.cos(yawRad),
                distance * Math.sin(yawRad),
                BALL_HEIGHT_METERS
            );

            // Transform to robot frame
            Translation3d ballInRobotFrame = ballInCamFrame
                .rotateBy(ROBOT_TO_DETECTION_CAM.getRotation())
                .plus(ROBOT_TO_DETECTION_CAM.getTranslation());

            // Transform to field frame
            double dx = ballInRobotFrame.getX() * Math.cos(heading)
                      - ballInRobotFrame.getY() * Math.sin(heading);
            double dy = ballInRobotFrame.getX() * Math.sin(heading)
                      + ballInRobotFrame.getY() * Math.cos(heading);

            ballPositions.add(new Translation3d(
                robotPose.getX() + dx,
                robotPose.getY() + dy,
                BALL_HEIGHT_METERS
            ));
        }

        return ballPositions;
    }

    /**
     * Returns the closest detected ball's field-relative position, if any.
     * Convenience wrapper around getAllBallFieldTranslations().
     */
    public Optional<Translation3d> getBallFieldRelativeTranslation() {
        return getAllBallFieldTranslations().stream().findFirst();
    }
}