package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose3d;

public class QuestResult {
    private final Pose3d pose;
    private final double timeStamp;

    public QuestResult(Pose3d pose, double timeStamp) {
        this.pose = pose;
        this.timeStamp = timeStamp;
    }

    public Pose3d getPose() {
        return pose;
    }

    public double getTimeStamp() {
        return timeStamp;
    }
}

