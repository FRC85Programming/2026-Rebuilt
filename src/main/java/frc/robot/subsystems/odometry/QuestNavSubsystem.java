package frc.robot.subsystems.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.QuestADBWrapper;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    // Object to handle all communication with NT4
    private final QuestNav questNav = new QuestNav();

    // The position of the quest relative to the robot's center
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-3.143),
                Units.inchesToMeters(-10.29),
                Units.inchesToMeters(16.59)
            ),
            new Rotation3d(0.0, 0.0, Math.toRadians(-94))
        );

    private int questNavRestartAttempts = 0;

    // Values that indicate how much to trust the quest
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02,
            0.02,
            0.035
        );

    @Override
    public void periodic() {
        if (!questNav.isConnected()) {
                if (questNavRestartAttempts < 25) {
                    questNavRestartAttempts++;
                    if (QuestADBWrapper.updateIsConnected()) { // QuestNav is not active but Quest ADB is connected, so try to restart QuestNav.
                        System.out.println("Connected, running restart...");
                        QuestADBWrapper.tryRestartQuestNav();
                    } else {
                        System.out.println("Disconnected, trying to reconnect lazily...");
                        QuestADBWrapper.lazyTryConnect();
                    }
                }
            }
    }

    /**
     * Gets the latest data from the quest
     * 
     * @return QuestResult, The quest data
     */
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

    /**
     * Gets unused data from the quest
     */
    public PoseFrame[] getAllUnreadPoseFrames() {
        return questNav.getAllUnreadPoseFrames();
    }

    /**
     * @return Matrix, the quest std devs
     */
    public Matrix<N3, N1> getStdDevs() {
        return QUESTNAV_STD_DEVS;
    }

    /**
     * Sets the position of the quest
     * 
     * @param robotPose The ROBOT POSITION that the quest will be set at
     */
    public void setPose(Pose3d robotPose) {
        Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);

        questNav.setPose(questPose);
    }

    public boolean isQuestConnected() {
        return questNav.isConnected();
    }
}
