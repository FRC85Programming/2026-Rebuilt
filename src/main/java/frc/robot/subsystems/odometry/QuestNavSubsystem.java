package frc.robot.subsystems.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    QuestNav questNav = new QuestNav();
    
    Transform3d ROBOT_TO_QUEST = new Transform3d(
        new edu.wpi.first.math.geometry.Translation3d(0.0, 0.0, 0.0),
        new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, 0.0) 
    );
    
    Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02,
        0.02, 
        0.035 
    );

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    public QuestResult getResult(PoseFrame questFrame) {

        if (questFrame.isTracking()) {
            Pose3d questPose = questFrame.questPose3d();

            double timestamp = questFrame.dataTimestamp();

            Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

            return new QuestResult(robotPose, timestamp);
        }

        DriverStation.reportWarning("No valid Quest pose frames available!", false); 
        return new QuestResult(new Pose3d(), 0.0);
    }

    public PoseFrame[] getAllUnreadPoseFrames() {
        return questNav.getAllUnreadPoseFrames();
    }

    public Matrix<N3, N1> getStdDevs() {
        return QUESTNAV_STD_DEVS;
    }
}
