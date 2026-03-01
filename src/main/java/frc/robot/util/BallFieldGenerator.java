package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * BallFieldGenerator
 *
 * Procedurally generates a randomized field of fuel balls modeled after
 * the three-zone configuration used in testing:
 *
 *   LEFT ZONE   |   CENTER ZONE   |   RIGHT ZONE
 *
 * Each call to getBalls() randomly assigns which of the three zones
 * contains the large, dense "main" clusters. The other two zones get
 * smaller, sparser "secondary" clusters. This prevents the robot from
 * hard-coding an assumption that the best balls are always in the center.
 *
 * Each call to getBalls() returns a freshly randomized array.
 * Pass a fixed seed to the constructor for reproducible results.
 *
 * Usage:
 *   // Random each time:
 *   BallFieldGenerator gen = new BallFieldGenerator();
 *
 *   // Reproducible:
 *   BallFieldGenerator gen = new BallFieldGenerator(12345L);
 *
 *   Translation2d[] balls = gen.getBalls();
 */
public class BallFieldGenerator {

    // -------------------------------------------------------------------------
    // Field geometry configuration
    // -------------------------------------------------------------------------

    /** X center of the entire field region (meters). */
    private static final double FIELD_CENTER_X = 8.25;

    /** Y center of the entire field region (meters). */
    private static final double FIELD_CENTER_Y = 4.0;

    /**
     * How far the side zones are offset from the center in X.
     * Right zone = FIELD_CENTER_X + SIDE_X_OFFSET
     * Left  zone = FIELD_CENTER_X - SIDE_X_OFFSET
     */
    private static final double SIDE_X_OFFSET = 1.95;

    /** Total Y span of the field (meters). Clusters are distributed across this. */
    private static final double FIELD_Y_SPAN = 6.0;

    /** Minimum physical gap between any two ball centers (ball diameter = 6in = 0.152m). */
    private static final double MIN_BALL_SPACING = 0.17;

    // -------------------------------------------------------------------------
    // Main zone cluster configuration
    // -------------------------------------------------------------------------

    /** Number of large clusters in the main zone. */
    private static final int MAIN_CLUSTER_COUNT = 3;

    /** Min/max balls per main cluster. */
    private static final int MAIN_CLUSTER_MIN_BALLS = 25;
    private static final int MAIN_CLUSTER_MAX_BALLS = 35;

    /**
     * Radius within which balls in a main cluster are scattered
     * around the cluster center (meters).
     */
    private static final double MAIN_CLUSTER_RADIUS = 0.50;

    /**
     * How much the cluster center can randomly drift from the
     * evenly-spaced Y position (meters).
     */
    private static final double MAIN_CLUSTER_Y_JITTER = 0.30;

    /**
     * How much the cluster center can randomly drift in the X direction (meters).
     */
    private static final double MAIN_CLUSTER_X_JITTER = 0.25;

    /** Number of scattered stragglers across the main zone. */
    private static final int MAIN_STRAGGLER_COUNT = 45;

    /** X spread of stragglers around the main zone center (meters). */
    private static final double MAIN_STRAGGLER_X_SPREAD = 0.80;

    // -------------------------------------------------------------------------
    // Secondary zone cluster configuration
    // -------------------------------------------------------------------------

    /** Number of small clusters per secondary zone. */
    private static final int SECONDARY_CLUSTER_COUNT = 3;

    /** Min/max balls per secondary cluster. Should be clearly less than main. */
    private static final int SECONDARY_CLUSTER_MIN_BALLS = 8;
    private static final int SECONDARY_CLUSTER_MAX_BALLS = 13;

    /** Radius within which balls in a secondary cluster are scattered (meters). */
    private static final double SECONDARY_CLUSTER_RADIUS = 0.28;

    /** Y jitter for secondary cluster centers (meters). */
    private static final double SECONDARY_CLUSTER_Y_JITTER = 0.25;

    /** X jitter for secondary cluster centers (meters). */
    private static final double SECONDARY_CLUSTER_X_JITTER = 0.15;

    /**
     * Number of sparse bridge stragglers between each secondary zone
     * and the main zone. Just enough to tempt, not enough to justify a detour.
     */
    private static final int BRIDGE_STRAGGLER_COUNT = 10;

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    private final Random random;

    /**
     * Which zone was chosen as the main (large cluster) zone in the last
     * getBalls() call. 0 = left, 1 = center, 2 = right.
     * Useful for logging/debugging.
     */
    private int lastMainZoneIndex = -1;

    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    /** Creates a generator with a random seed — different result each call. */
    public BallFieldGenerator() {
        this.random = new Random();
    }

    /**
     * Creates a generator with a fixed seed — same result every call.
     * Useful for repeatable testing.
     *
     * @param seed Any long value.
     */
    public BallFieldGenerator(long seed) {
        this.random = new Random(seed);
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Generates and returns a randomized array of ball positions.
     *
     * Each call randomly picks which of the three zones (left, center, right)
     * holds the large dense main clusters. The other two zones get small
     * secondary clusters, with sparse bridge stragglers between zones.
     *
     * @return Array of Translation2d ball positions in field-relative coordinates.
     */
    public Translation2d[] getBalls() {
        List<Translation2d> balls = new ArrayList<>();

        // The three zone X centers, indexed 0=left, 1=center, 2=right
        double[] zoneCentersX = {
            FIELD_CENTER_X - SIDE_X_OFFSET,
            FIELD_CENTER_X,
            FIELD_CENTER_X + SIDE_X_OFFSET
        };

        // Randomly pick which zone gets the main clusters this round
        lastMainZoneIndex = random.nextInt(3);
        double mainX = zoneCentersX[lastMainZoneIndex];

        // Evenly space cluster Y centers across the field Y span
        double yPadding = FIELD_Y_SPAN / (MAIN_CLUSTER_COUNT + 1);
        double yBottom  = FIELD_CENTER_Y - (FIELD_Y_SPAN / 2.0);

        double[] clusterYCenters = new double[MAIN_CLUSTER_COUNT];
        for (int i = 0; i < MAIN_CLUSTER_COUNT; i++) {
            double baseY = yBottom + yPadding * (i + 1);
            clusterYCenters[i] = baseY + jitter(MAIN_CLUSTER_Y_JITTER);
        }

        // --- Main zone clusters ---
        for (int i = 0; i < MAIN_CLUSTER_COUNT; i++) {
            double cx = mainX + jitter(MAIN_CLUSTER_X_JITTER);
            double cy = clusterYCenters[i];
            int count = randInt(MAIN_CLUSTER_MIN_BALLS, MAIN_CLUSTER_MAX_BALLS);
            generateCluster(balls, cx, cy, count, MAIN_CLUSTER_RADIUS);
        }

        // --- Main zone stragglers ---
        for (int i = 0; i < MAIN_STRAGGLER_COUNT; i++) {
            double x = mainX + jitter(MAIN_STRAGGLER_X_SPREAD);
            double y = yBottom + random.nextDouble() * FIELD_Y_SPAN;
            addIfClear(balls, x, y);
        }

        // --- Secondary zones: the two zones that didn't get the main clusters ---
        for (int zoneIdx = 0; zoneIdx < 3; zoneIdx++) {
            if (zoneIdx == lastMainZoneIndex) continue;

            double secondaryX = zoneCentersX[zoneIdx];

            // Determine bridge X range: from halfway between this zone and the
            // main zone, toward the secondary zone edge
            double halfwayX = (secondaryX + mainX) / 2.0;
            double bridgeStartX = halfwayX;
            // Pull the bridge end slightly inside the secondary zone
            double bridgeEndX = secondaryX + (mainX > secondaryX ? 0.15 : -0.15);

            generateSecondaryZone(balls, secondaryX, clusterYCenters, yBottom,
                                  bridgeStartX, bridgeEndX);
        }

        return balls.toArray(new Translation2d[0]);
    }

    /**
     * Returns a human-readable summary of the last generated field.
     * Useful for SmartDashboard logging or match notes.
     */
    public String getSummary(Translation2d[] balls) {
        String[] zoneNames = {"LEFT", "CENTER", "RIGHT"};
        String mainZoneName = (lastMainZoneIndex >= 0) ? zoneNames[lastMainZoneIndex] : "UNKNOWN";
        return String.format(
            "BallFieldGenerator: %d total balls | main zone=%s | center=(%.2f, %.2f) | Y span=%.1fm | side offset=±%.2fm",
            balls.length, mainZoneName, FIELD_CENTER_X, FIELD_CENTER_Y, FIELD_Y_SPAN, SIDE_X_OFFSET
        );
    }

    /**
     * Returns which zone (0=left, 1=center, 2=right) was assigned the main
     * clusters in the most recent getBalls() call. Returns -1 if getBalls()
     * has not been called yet.
     */
    public int getLastMainZoneIndex() {
        return lastMainZoneIndex;
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Generates a full secondary zone: clusters at roughly the same Y positions
     * as the main clusters, plus sparse bridge stragglers linking back to the main zone.
     */
    private void generateSecondaryZone(List<Translation2d> balls,
                                       double zoneCenterX,
                                       double[] mainClusterYs,
                                       double yBottom,
                                       double bridgeXStart,
                                       double bridgeXEnd) {
        // Secondary clusters — aligned roughly with main cluster Y positions
        // but with their own jitter so they don't line up perfectly
        for (int i = 0; i < SECONDARY_CLUSTER_COUNT; i++) {
            int mainIdx = (int) Math.round((double) i * (MAIN_CLUSTER_COUNT - 1)
                                           / Math.max(SECONDARY_CLUSTER_COUNT - 1, 1));
            double cy = mainClusterYs[mainIdx] + jitter(SECONDARY_CLUSTER_Y_JITTER);
            double cx = zoneCenterX + jitter(SECONDARY_CLUSTER_X_JITTER);
            int count = randInt(SECONDARY_CLUSTER_MIN_BALLS, SECONDARY_CLUSTER_MAX_BALLS);
            generateCluster(balls, cx, cy, count, SECONDARY_CLUSTER_RADIUS);
        }

        // Bridge stragglers — sparse balls scattered between the secondary zone
        // and the main zone boundary
        double xMin = Math.min(bridgeXStart, bridgeXEnd);
        double xMax = Math.max(bridgeXStart, bridgeXEnd);
        double xRange = xMax - xMin;

        for (int i = 0; i < BRIDGE_STRAGGLER_COUNT; i++) {
            double x = xMin + random.nextDouble() * xRange;
            double y = yBottom + random.nextDouble() * FIELD_Y_SPAN;
            addIfClear(balls, x, y);
        }
    }

    /**
     * Generates a single cluster of balls scattered around a center point.
     * Uses rejection sampling to ensure no two balls overlap.
     *
     * @param balls     The list to add balls to.
     * @param cx        Cluster center X.
     * @param cy        Cluster center Y.
     * @param count     Target number of balls to place.
     * @param radius    Max scatter radius from center.
     */
    private void generateCluster(List<Translation2d> balls,
                                 double cx, double cy,
                                 int count, double radius) {
        int attempts = 0;
        int placed   = 0;
        int maxAttempts = count * 20;

        while (placed < count && attempts < maxAttempts) {
            double angle = random.nextDouble() * 2.0 * Math.PI;
            double dist  = Math.sqrt(random.nextDouble()) * radius;
            double x = cx + dist * Math.cos(angle);
            double y = cy + dist * Math.sin(angle);

            if (addIfClear(balls, x, y)) {
                placed++;
            }
            attempts++;
        }
    }

    /**
     * Adds a ball at (x, y) only if no existing ball is within MIN_BALL_SPACING.
     *
     * @return true if the ball was added, false if it was too close to an existing one.
     */
    private boolean addIfClear(List<Translation2d> balls, double x, double y) {
        Translation2d candidate = new Translation2d(x, y);
        for (Translation2d existing : balls) {
            if (existing.getDistance(candidate) < MIN_BALL_SPACING) {
                return false;
            }
        }
        balls.add(candidate);
        return true;
    }

    /**
     * Returns a random double in [-maxJitter, +maxJitter].
     */
    private double jitter(double maxJitter) {
        return (random.nextDouble() * 2.0 - 1.0) * maxJitter;
    }

    /**
     * Returns a random int in [min, max] inclusive.
     */
    private int randInt(int min, int max) {
        return min + random.nextInt(max - min + 1);
    }
}