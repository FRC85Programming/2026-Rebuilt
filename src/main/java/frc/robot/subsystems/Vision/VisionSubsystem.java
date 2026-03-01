package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {

    private static final String DETECTION_CAMERA_NAME = "Balls";

    private static final Transform3d ROBOT_TO_DETECTION_CAM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.5),
                Units.inchesToMeters(7.45),
                Units.inchesToMeters(18)
            ),
            new Rotation3d(0.0, Units.degreesToRadians(9.0), 0.0)
        );

    private static final double DETECTION_CAM_HEIGHT_METERS = Units.inchesToMeters(10.5);
    private static final double DETECTION_CAM_PITCH_DEGREES = 9.0;
    private static final double BALL_HEIGHT_METERS = 0.075;
    private static final int FUEL_CLASS_ID = 0;
    private static final double MIN_DETECTION_CONFIDENCE = 0.3;

    // Median filter window — smooths camera-frame yaw/distance before field transform
    private static final int MEDIAN_WINDOW_SIZE = 30;

    private final PhotonCamera detectionCamera;
    private Supplier<Pose2d> poseSupplier = null;

    // History stored in camera-relative polar coords (distance, yaw) to avoid drift
    private final Deque<double[]> camHistory = new ArrayDeque<>();

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
            getSmoothedBallFieldTranslation().ifPresentOrElse(
                t -> Logger.recordOutput("Vision/BallPosition3d", new Translation3d[] { t }),
                () -> {
                    camHistory.clear();
                    Logger.recordOutput("Vision/BallPosition3d", new Translation3d[] {});
                }
            );
        }
    }

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

    /** Returns the median-filtered, field-relative ball Translation3d. */
    public Optional<Translation3d> getSmoothedBallFieldTranslation() {
        if (poseSupplier == null) return Optional.empty();

        var result = detectionCamera.getLatestResult();
        if (!result.hasTargets()) return Optional.empty();

        var bestTarget = result.getTargets().stream()
            .filter(t -> t.getDetectedObjectClassID() == FUEL_CLASS_ID)
            .filter(t -> t.getDetectedObjectConfidence() >= MIN_DETECTION_CONFIDENCE)
            .findFirst();

        if (bestTarget.isEmpty()) return Optional.empty();

        var t = bestTarget.get();
        double distance = Math.abs(PhotonUtils.calculateDistanceToTargetMeters(
            DETECTION_CAM_HEIGHT_METERS,
            BALL_HEIGHT_METERS,
            Units.degreesToRadians(DETECTION_CAM_PITCH_DEGREES),
            Units.degreesToRadians(-t.getPitch())
        ));
        double yawDeg = -t.getYaw();

        // Store camera-relative [distance, yaw] — not field-relative, so robot motion won't corrupt history
        camHistory.addLast(new double[]{ distance, yawDeg });
        if (camHistory.size() > MEDIAN_WINDOW_SIZE) camHistory.removeFirst();

        // Median filter over distance and yaw independently
        List<Double> distances = new ArrayList<>();
        List<Double> yaws = new ArrayList<>();
        for (double[] sample : camHistory) {
            distances.add(sample[0]);
            yaws.add(sample[1]);
        }
        Collections.sort(distances);
        Collections.sort(yaws);

        int mid = camHistory.size() / 2;
        double medianDistance = distances.get(mid);
        double medianYawRad = Units.degreesToRadians(yaws.get(mid));

        // Decompose into camera-frame Translation3d
        Translation3d ballInCamFrame = new Translation3d(
            medianDistance * Math.cos(medianYawRad),
            medianDistance * Math.sin(medianYawRad),
            BALL_HEIGHT_METERS
        );

        // Transform to robot frame
        Translation3d ballInRobotFrame = ballInCamFrame
            .rotateBy(ROBOT_TO_DETECTION_CAM.getRotation())
            .plus(ROBOT_TO_DETECTION_CAM.getTranslation());

        // Transform to field frame using current robot pose
        Pose2d robotPose = poseSupplier.get();
        double heading = robotPose.getRotation().getRadians();
        double dx = ballInRobotFrame.getX() * Math.cos(heading)
                  - ballInRobotFrame.getY() * Math.sin(heading);
        double dy = ballInRobotFrame.getX() * Math.sin(heading)
                  + ballInRobotFrame.getY() * Math.cos(heading);

        return Optional.of(new Translation3d(
            robotPose.getX() + dx,
            robotPose.getY() + dy,
            BALL_HEIGHT_METERS
        ));
    }

    public Optional<Translation3d> getBallFieldRelativeTranslation() {
        return getSmoothedBallFieldTranslation();
    }
}