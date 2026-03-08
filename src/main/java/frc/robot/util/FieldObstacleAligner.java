package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ObstacleAlignmentConstants;
import java.util.function.BooleanSupplier;

/**
 * Passive trench Y-centering helper.
 *
 * When the robot is within {@link ObstacleAlignmentConstants#DETECTION_RANGE} meters (in X) of a
 * trench and moving toward it, the driver's {@code vy} is replaced with a PID output that centers
 * the robot at the trench's configurable target Y. All other inputs are passed through unchanged.
 *
 * A SmartDashboard boolean key {@code "Obstacle Alignment Enabled"} (default {@code true}) can be
 * used to disable the feature at runtime.
 */
public class FieldObstacleAligner {

  /**
   * Immutable description of a trench zone on the field.
   *
   * @param xMin    Left edge of the box (meters).
   * @param xMax    Right edge of the box (meters).
   * @param yMin    Bottom edge of the box (meters).
   * @param yMax    Top edge of the box (meters).
   * @param targetY Desired robot Y while traversing the trench.
   */
  public record TrenchBox(
      double xMin, double xMax,
      double yMin, double yMax,
      double targetY) {

    public boolean containsY(double y) {
      return y >= yMin && y <= yMax;
    }

    public double xDistanceFromEdge(double robotX) {
      if (robotX < xMin) return xMin - robotX;
      if (robotX > xMax) return robotX - xMax;
      return 0.0;
    }
  }

  private static final String SMARTDASHBOARD_KEY = "Obstacle Alignment Enabled";

  private final PIDController yPID;
  private final TrenchBox[] trenches;

  /**
   * Supplier that returns {@code true} when the pose source (QuestNav) is
   * actively tracking.  When it returns {@code false} this aligner — and any
   * future pose-dependent feature wired the same way — will fall back to
   * passing driver input through unchanged.
   */
  private final BooleanSupplier poseTracking;

  /**
   * Creates an aligner that is always enabled regardless of tracking state.
   * Prefer {@link #FieldObstacleAligner(BooleanSupplier)} when a QuestNav
   * subsystem is available.
   */
  public FieldObstacleAligner() {
    this(() -> true);
  }

  /**
   * Creates an aligner gated on the provided tracking supplier.
   *
   * @param poseTracking {@code true} when the pose source has a valid fix.
   *                     Pass {@code questNavSubsystem::isTracking} here.
   */
  public FieldObstacleAligner(BooleanSupplier poseTracking) {
    this.poseTracking = poseTracking;

    yPID = new PIDController(
        ObstacleAlignmentConstants.Y_KP,
        ObstacleAlignmentConstants.Y_KI,
        ObstacleAlignmentConstants.Y_KD);
    yPID.setTolerance(0.02);

    trenches = new TrenchBox[]{
        new TrenchBox(
            ObstacleAlignmentConstants.TRENCH1_X_MIN,
            ObstacleAlignmentConstants.TRENCH1_X_MAX,
            ObstacleAlignmentConstants.TRENCH1_Y_MIN,
            ObstacleAlignmentConstants.TRENCH1_Y_MAX,
            ObstacleAlignmentConstants.TRENCH1_TARGET_Y),
        new TrenchBox(
            ObstacleAlignmentConstants.TRENCH2_X_MIN,
            ObstacleAlignmentConstants.TRENCH2_X_MAX,
            ObstacleAlignmentConstants.TRENCH2_Y_MIN,
            ObstacleAlignmentConstants.TRENCH2_Y_MAX,
            ObstacleAlignmentConstants.TRENCH2_TARGET_Y)
    };

    SmartDashboard.putBoolean(SMARTDASHBOARD_KEY, true);
  }

  /**
   * Applies trench Y-centering to the driver's commanded speeds.
   *
   * @param pose        Current robot pose on the field.
   * @param driverInput Field-relative {@link ChassisSpeeds} from the driver.
   * @return Speeds with {@code vy} replaced by PID output when inside a trench's detection zone,
   *         otherwise the original input unchanged.
   */
  public ChassisSpeeds apply(Pose2d pose, ChassisSpeeds driverInput) {
    if (!poseTracking.getAsBoolean()) {
      SmartDashboard.putString("Obstacle Alignment", "No Tracking");
      return driverInput;
    }

    if (!SmartDashboard.getBoolean(SMARTDASHBOARD_KEY, true)) {
      SmartDashboard.putString("Obstacle Alignment", "Disabled");
      return driverInput;
    }

    double robotX = pose.getTranslation().getX();
    double robotY = pose.getTranslation().getY();
    double vx = driverInput.vxMetersPerSecond;

    for (TrenchBox box : trenches) {
      if (isActive(box, robotX, robotY, vx)) {
        double correctedVy = yPID.calculate(robotY, box.targetY());

        SmartDashboard.putString("Obstacle Alignment", "Trench");
        SmartDashboard.putNumber("Obstacle Y Target (m)", box.targetY());

        return new ChassisSpeeds(vx, correctedVy, driverInput.omegaRadiansPerSecond);
      }
    }

    SmartDashboard.putString("Obstacle Alignment", "None");
    return driverInput;
  }

  /**
   * Returns true when the robot should receive Y correction for the given trench.
   *
   * <p>Conditions (all must be true):
   * <ol>
   *   <li>Robot Y is inside the trench's Y range.
   *   <li>Robot is approaching in X (or already inside the X range).
   *   <li>Robot is within {@link ObstacleAlignmentConstants#DETECTION_RANGE} meters of the X edge.
   * </ol>
   */
  private boolean isActive(TrenchBox box, double robotX, double robotY, double vx) {
    if (!box.containsY(robotY)) return false;

    double xDist = box.xDistanceFromEdge(robotX);

    boolean insideX = robotX >= box.xMin() && robotX <= box.xMax();
    boolean approaching = insideX
        || (robotX < box.xMin() && vx > 0.0)
        || (robotX > box.xMax() && vx < 0.0);

    if (!approaching) return false;

    return xDist <= ObstacleAlignmentConstants.DETECTION_RANGE;
  }
}
