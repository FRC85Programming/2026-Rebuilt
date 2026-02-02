package frc.robot.subsystems.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNav questNav = new QuestNav();

    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14.5),
                0.0,
                Units.inchesToMeters(13)
            ),
            new Rotation3d(0.0, 0.0, 0.0)
        );

    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02,
            0.02,
            0.035
        );

    public QuestResult getResult(PoseFrame questFrame) {

        if (!questFrame.isTracking()) {
            DriverStation.reportWarning("No valid Quest pose frames available!", false);
            return new QuestResult(new Pose3d(), 0.0);
        }

        Pose3d questPose = questFrame.questPose3d();
        double timestamp = questFrame.dataTimestamp();

        Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

        return new QuestResult(robotPose, timestamp);
    }

    public PoseFrame[] getAllUnreadPoseFrames() {
        return questNav.getAllUnreadPoseFrames();
    }

    public Matrix<N3, N1> getStdDevs() {
        return QUESTNAV_STD_DEVS;
    }

    public void setPose(Pose3d robotPose) {
        Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

        questNav.setPose(questPose);
    }
}
