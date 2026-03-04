package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * FindBestBallPath
 *
 * Calculates the optimal single-pass path for the robot to collect
 * as many fuel balls as possible in one continuous drive-through.
 *
 * The robot always progresses in the sweep direction (Y axis), starting
 * from its actual field position. The DP is anchored to the robot's
 * starting X, so only physically reachable paths are considered.
 *
 * Usage:
 *   Translation2d[] balls = ...; // ball poses relative to field
 * 
 *   Translation2d robotPose = new Translation2d(3.0, 1.0); // robot start
 *   BallPathCalculator finder = new BallPathCalculator(balls, robotPose, 1.0);
 *   Translation2d[] path = finder.getBestPath();
 */
public class BallPathCalculator {

    // -------------------------------------------------------------------------
    // Tuning constants — adjust these to match your robot
    // -------------------------------------------------------------------------

    /** Width of the intake in meters (28 inches). */
    private static final double INTAKE_WIDTH_METERS = 0.711;

    /** Half the intake width — max perpendicular distance to collect a ball. */
    private static final double INTAKE_RADIUS = INTAKE_WIDTH_METERS / 2.0;

    /**
     * Number of horizontal row slices to divide the field into.
     * More rows = finer path resolution, but slower to compute.
     * 10-20 is a good range for a 5-second auto pass.
     */
    private static final int NUM_ROWS = 8;

    /**
     * Maximum X shift (meters) the robot can make between adjacent rows.
     * Tune this based on:
     *   max_lateral_speed (m/s) * time_per_row (s)
     * Example: 2.0 m/s lateral * 0.33s per row = ~0.66m
     */
    private static final double MAX_X_SHIFT_PER_ROW = 0.4;

    /**
     * Minimum number of balls a row must offer to bother shifting toward it.
     * Prevents the robot from making unnecessary micro-adjustments.
     */
    private static final int MIN_BALLS_TO_BOTHER = 1;

    /**
     * How far around each ball to generate candidate X positions (meters).
     * Candidate X values are generated at each ball's X, plus offsets at
     * +/- this value, to let the intake straddle nearby clusters.
     */
    private static final double CANDIDATE_X_SPREAD = INTAKE_RADIUS;

    /**
     * The robot's starting X is always included as a candidate and is the
     * mandatory anchor for row 0. Any candidate that is unreachable from
     * the robot start within (row index * MAX_X_SHIFT_PER_ROW) meters is
     * pruned early so the DP never selects it.
     */

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** All known ball positions on the field. */
    private final Translation2d[] ballPoses;

    /** The robot's starting position on the field. */
    private final Translation2d robotStart;

    /**
     * Sweep direction along Y axis.
     * Positive = robot drives in +Y direction.
     * Negative = robot drives in -Y direction.
     */
    private final double sweepDirection;

    /**
     * Hard Y boundary — the path will not extend past this value.
     * For a +Y sweep: balls with Y > sweepYLimit are ignored.
     * For a -Y sweep: balls with Y < sweepYLimit are ignored.
     * Defaults to no limit (positive infinity / negative infinity).
     */
    private final double sweepYLimit;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * @param ballPoses      Array of ball positions in field-relative coordinates.
     * @param robotStart     The robot's starting position in field-relative coordinates.
     * @param sweepDirection +1.0 to sweep in the +Y direction, -1.0 for -Y.
     */
    public BallPathCalculator(Translation2d[] ballPoses, Translation2d robotStart, double sweepDirection) {
        this(ballPoses, robotStart, sweepDirection,
             Math.signum(sweepDirection) > 0 ? Double.POSITIVE_INFINITY : Double.NEGATIVE_INFINITY);
    }

    /**
     * @param ballPoses      Array of ball positions in field-relative coordinates.
     * @param robotStart     The robot's starting position in field-relative coordinates.
     * @param sweepDirection +1.0 to sweep in the +Y direction, -1.0 for -Y.
     * @param sweepYLimit    Hard Y boundary — balls past this Y are ignored.
     *                       For +Y sweep: balls with Y > sweepYLimit are excluded.
     *                       For -Y sweep: balls with Y < sweepYLimit are excluded.
     */
    public BallPathCalculator(Translation2d[] ballPoses, Translation2d robotStart,
                              double sweepDirection, double sweepYLimit) {
        this.ballPoses = ballPoses;
        this.robotStart = robotStart;
        this.sweepDirection = Math.signum(sweepDirection); // normalize to exactly +1 or -1
        this.sweepYLimit = sweepYLimit;
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /**
     * Calculates and returns the optimal path as an array of (x, y) waypoints.
     *
     * The path always starts near the robot's actual position and progresses
     * in the sweep direction. Only X shifts that are physically reachable
     * row-by-row are considered.
     *
     * @return Array of Translation2d waypoints. Empty if no balls are found.
     */
    public Translation2d[] getBestPath() {
        if (ballPoses == null || ballPoses.length == 0) {
            return new Translation2d[0];
        }

        // Step 1: Find the Y range of all balls *ahead of the robot* and slice into rows.
        // We only care about balls in the sweep direction from the robot's start.
        double robotY = robotStart.getY();
        double[] yRange = getYRangeAhead(robotY);

        if (yRange == null) {
            // No balls are ahead of the robot in the sweep direction
            return new Translation2d[0];
        }

        double[] rowCenters = buildBallAwareRows(robotY);

        if (rowCenters.length == 0) {
            return new Translation2d[0];
        }

        // Step 2: Build candidate X positions from ball locations + robot start X
        double[] candidateXs = buildCandidateXs();

        if (candidateXs.length == 0) {
            return new Translation2d[0];
        }

        // Step 3: Run dynamic programming to find best X at each row.
        // Row 0 is anchored to the robot's actual X — only candidates reachable
        // from robotStart.getX() within one row's lateral budget are valid there.
        int numRows = rowCenters.length;
        int numXs = candidateXs.length;

        // dp[row][xIdx] = best total balls collected arriving at this (row, x)
        double[][] dp = new double[numRows][numXs];
        int[][] parent = new int[numRows][numXs];

        // Sentinel: -1 means "this state is unreachable"
        final double UNREACHABLE = Double.NEGATIVE_INFINITY;

        for (int r = 0; r < numRows; r++) {
            for (int x = 0; x < numXs; x++) {
                dp[r][x] = UNREACHABLE;
                parent[r][x] = -1;
            }
        }

        // Fill row 0: only X values reachable from the robot's starting X.
        // We treat the segment from (robotStart.X, robotStart.Y) to (candidateX, rowCenters[0])
        // as the first leg, so we also score any balls swept along that approach segment.
        for (int xIdx = 0; xIdx < numXs; xIdx++) {
            double xShiftFromStart = Math.abs(candidateXs[xIdx] - robotStart.getX());
            if (xShiftFromStart > MAX_X_SHIFT_PER_ROW) {
                // Can't reach this X from the robot's starting position by row 0
                continue;
            }

            // Score the approach segment: from robot start to (candidateX, rowCenter[0])
            double ballsOnApproach = countBallsAlongSegment(
                robotStart.getX(), robotY,
                candidateXs[xIdx], rowCenters[0]
            );
            dp[0][xIdx] = ballsOnApproach;
        }

        // Fill remaining rows — same DP as before, but UNREACHABLE states propagate
        for (int row = 1; row < numRows; row++) {
            for (int xIdx = 0; xIdx < numXs; xIdx++) {
                double bestScore = UNREACHABLE;
                int bestParent = -1;

                for (int prevXIdx = 0; prevXIdx < numXs; prevXIdx++) {
                    if (dp[row - 1][prevXIdx] == UNREACHABLE) continue;

                    double xShift = Math.abs(candidateXs[xIdx] - candidateXs[prevXIdx]);
                    if (xShift > MAX_X_SHIFT_PER_ROW) continue;

                    double ballsOnSegment = countBallsAlongSegment(
                        candidateXs[prevXIdx], rowCenters[row - 1],
                        candidateXs[xIdx],     rowCenters[row]
                    );

                    double totalScore = dp[row - 1][prevXIdx] + ballsOnSegment;

                    if (totalScore > bestScore) {
                        bestScore = totalScore;
                        bestParent = prevXIdx;
                    }
                }

                dp[row][xIdx] = bestScore;
                parent[row][xIdx] = bestParent;
            }
        }

        // Step 4: Find the best reachable ending X in the last row
        int bestFinalXIdx = -1;
        for (int xIdx = 0; xIdx < numXs; xIdx++) {
            if (dp[numRows - 1][xIdx] == UNREACHABLE) continue;
            if (bestFinalXIdx == -1 || dp[numRows - 1][xIdx] > dp[numRows - 1][bestFinalXIdx]) {
                bestFinalXIdx = xIdx;
            }
        }

        if (bestFinalXIdx == -1) {
            // Every path was unreachable — shouldn't happen if row 0 had valid states
            return new Translation2d[0];
        }

        // Step 5: Trace back through parent pointers to reconstruct path
        List<Translation2d> path = new ArrayList<>();
        int currentXIdx = bestFinalXIdx;

        for (int row = numRows - 1; row >= 0; row--) {
            path.add(0, new Translation2d(candidateXs[currentXIdx], rowCenters[row]));
            if (row > 0 && parent[row][currentXIdx] >= 0) {
                currentXIdx = parent[row][currentXIdx];
            }
        }

        // Step 6: Simplify path — remove waypoints where X didn't meaningfully change
        path = simplifyPath(path);

        return path.toArray(new Translation2d[0]);
    }

    /**
     * Returns the estimated number of balls the best path will collect.
     * Useful for logging or deciding whether to run this auto at all.
     */
    public int getEstimatedBallCount() {
        Translation2d[] path = getBestPath();
        if (path.length == 0) return 0;

        int count = 0;
        List<Translation2d> collected = new ArrayList<>();

        for (int i = 0; i < path.length - 1; i++) {
            for (Translation2d ball : ballPoses) {
                if (!collected.contains(ball)) {
                    if (distanceToSegment(ball, path[i], path[i + 1]) <= INTAKE_RADIUS) {
                        collected.add(ball);
                        count++;
                    }
                }
            }
        }
        return count;
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * Returns [yMin, yMax] for all balls that are ahead of the robot
     * in the sweep direction. Returns null if no balls are ahead.
     */
    private double[] getYRangeAhead(double robotY) {
        double yMin = Double.MAX_VALUE;
        double yMax = Double.MIN_VALUE;
        boolean found = false;

        for (Translation2d ball : ballPoses) {
            double by = ball.getY();
            // "Ahead" means further in the sweep direction than the robot
            if (sweepDirection > 0 && by < robotY) continue;
            if (sweepDirection < 0 && by > robotY) continue;
            // Respect the hard Y limit
            if (sweepDirection > 0 && by > sweepYLimit) continue;
            if (sweepDirection < 0 && by < sweepYLimit) continue;

            yMin = Math.min(yMin, by);
            yMax = Math.max(yMax, by);
            found = true;
        }

        return found ? new double[]{yMin, yMax} : null;
    }

    /**
     * Maximum Y distance between two balls for them to be merged into the
     * same cluster row. Tune alongside ball spacing on the field.
     */
    private static final double CLUSTER_EPSILON_METERS = 0.3;

    /**
     * Builds row anchor Y positions by clustering ball Y coordinates.
     *
     * Steps:
     *   1. Collect Y positions of all balls ahead of the robot.
     *   2. Sort them in travel order (ascending for +Y sweep, descending for -Y).
     *   3. Merge consecutive balls within CLUSTER_EPSILON_METERS into one cluster
     *      and use the cluster's mean Y as the row anchor.
     *   4. Prepend a "row 0" anchor just past the robot start so the DP always
     *      has an initial scoring segment before the first ball cluster.
     *
     * Falls back to NUM_ROWS equal slices when fewer than 2 distinct clusters
     * are found (e.g. all balls at the same Y).
     *
     * @param robotY  The robot's current Y position.
     * @return Array of row anchor Y values in travel order (nearest → farthest).
     */
    private double[] buildBallAwareRows(double robotY) {
        // --- 1. Collect relevant ball Y positions ---
        List<Double> ballYs = new ArrayList<>();
        for (Translation2d ball : ballPoses) {
            double by = ball.getY();
            if (sweepDirection > 0 && by < robotY) continue;
            if (sweepDirection < 0 && by > robotY) continue;
            if (sweepDirection > 0 && by > sweepYLimit) continue;
            if (sweepDirection < 0 && by < sweepYLimit) continue;
            ballYs.add(by);
        }

        if (ballYs.isEmpty()) {
            // No balls ahead — caller already handles this, but return safe default
            return new double[0];
        }

        // --- 2. Sort in travel order ---
        if (sweepDirection > 0) {
            ballYs.sort(Double::compareTo);          // ascending
        } else {
            ballYs.sort((a, b) -> Double.compare(b, a)); // descending
        }

        // --- 3. Cluster consecutive balls within CLUSTER_EPSILON_METERS ---
        List<Double> clusterCenters = new ArrayList<>();
        List<Double> currentCluster = new ArrayList<>();
        currentCluster.add(ballYs.get(0));

        for (int i = 1; i < ballYs.size(); i++) {
            double prev = ballYs.get(i - 1);
            double curr = ballYs.get(i);
            if (Math.abs(curr - prev) <= CLUSTER_EPSILON_METERS) {
                currentCluster.add(curr);
            } else {
                double mean = 0;
                for (double v : currentCluster) mean += v;
                clusterCenters.add(mean / currentCluster.size());
                currentCluster.clear();
                currentCluster.add(curr);
            }
        }
        // Flush last cluster
        double mean = 0;
        for (double v : currentCluster) mean += v;
        clusterCenters.add(mean / currentCluster.size());

        // --- 4. Fall back to equal slices when there is only one cluster ---
        if (clusterCenters.size() < 2) {
            double yEnd = sweepDirection > 0 ? ballYs.get(ballYs.size() - 1) : ballYs.get(0);
            double rowHeight = Math.abs(yEnd - robotY) / NUM_ROWS;
            double[] fallback = new double[NUM_ROWS];
            for (int i = 0; i < NUM_ROWS; i++) {
                fallback[i] = robotY + sweepDirection * (i + 0.5) * rowHeight;
            }
            return fallback;
        }

        // Prepend a "pre-cluster" anchor halfway between the robot and the first cluster,
        // so the approach segment from the robot's position is scored by the DP.
        double firstCluster = clusterCenters.get(0);
        double approachAnchor = (robotY + firstCluster) / 2.0;

        double[] rows = new double[clusterCenters.size() + 1];
        rows[0] = approachAnchor;
        for (int i = 0; i < clusterCenters.size(); i++) {
            rows[i + 1] = clusterCenters.get(i);
        }
        return rows;
    }

    /**
     * Generates candidate X positions to consider at each row.
     * Always includes the robot's starting X as a candidate so the
     * DP always has a valid anchor at row 0.
     */
    private double[] buildCandidateXs() {
        List<Double> candidates = new ArrayList<>();

        // Always include robot's starting X — this is the row-0 anchor
        addIfNew(candidates, robotStart.getX());

        for (Translation2d ball : ballPoses) {
            double x = ball.getX();
            addIfNew(candidates, x);
            addIfNew(candidates, x + CANDIDATE_X_SPREAD);
            addIfNew(candidates, x - CANDIDATE_X_SPREAD);
        }

        candidates.sort(Double::compareTo);
        double[] arr = new double[candidates.size()];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = candidates.get(i);
        }
        return arr;
    }

    /**
     * Adds an X value to the list if no existing value is within 2cm of it.
     * Prevents near-duplicate candidates from cluttering the DP table.
     */
    private void addIfNew(List<Double> list, double value) {
        for (double existing : list) {
            if (Math.abs(existing - value) < 0.02) return;
        }
        list.add(value);
    }

    /**
     * Counts how many balls are within INTAKE_RADIUS of the line segment
     * from (x1, y1) to (x2, y2).
     */
    private double countBallsAlongSegment(double x1, double y1, double x2, double y2) {
        int count = 0;
        Translation2d segStart = new Translation2d(x1, y1);
        Translation2d segEnd   = new Translation2d(x2, y2);

        for (Translation2d ball : ballPoses) {
            if (distanceToSegment(ball, segStart, segEnd) <= INTAKE_RADIUS) {
                count++;
            }
        }
        return count;
    }

    /**
     * Calculates the perpendicular distance from a point to a line segment.
     */
    private double distanceToSegment(Translation2d point, Translation2d segStart, Translation2d segEnd) {
        double dx = segEnd.getX() - segStart.getX();
        double dy = segEnd.getY() - segStart.getY();
        double lengthSq = dx * dx + dy * dy;

        if (lengthSq < 1e-9) {
            return point.getDistance(segStart);
        }

        double t = ((point.getX() - segStart.getX()) * dx
                  + (point.getY() - segStart.getY()) * dy)
                  / lengthSq;
        t = Math.max(0, Math.min(1, t));

        Translation2d closest = new Translation2d(
            segStart.getX() + t * dx,
            segStart.getY() + t * dy
        );

        return point.getDistance(closest);
    }

    /**
     * Removes redundant waypoints where the X value didn't meaningfully change.
     * Keeps the first and last point always.
     */
    private List<Translation2d> simplifyPath(List<Translation2d> path) {
        if (path.size() <= 2) return path;

        List<Translation2d> simplified = new ArrayList<>();
        simplified.add(path.get(0));

        for (int i = 1; i < path.size() - 1; i++) {
            double prevX = path.get(i - 1).getX();
            double currX = path.get(i).getX();
            double nextX = path.get(i + 1).getX();

            boolean xChangingNow  = Math.abs(currX - prevX) > 0.05;
            boolean xChangingNext = Math.abs(nextX - currX) > 0.05;

            if (xChangingNow || xChangingNext) {
                simplified.add(path.get(i));
            }
        }

        simplified.add(path.get(path.size() - 1));
        return simplified;
    }
}