package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * BallPathCalculator
 *
 * Calculates the optimal single-pass path for the robot to collect
 * as many fuel balls as possible in one continuous drive-through.
 *
 * Instead of dividing the field into a fixed grid of rows, this version
 * clusters balls by Y proximity and uses one DP row per cluster. This
 * eliminates degenerate behavior with few balls (no arbitrary 8-row
 * subdivision of short distances) and naturally scales with ball count.
 *
 * A lateral movement penalty ensures the DP prefers straighter paths
 * when multiple paths collect the same number of balls.
 *
 * Usage:
 *   Translation2d[] balls = ...;
 *   Translation2d robotPose = new Translation2d(3.0, 1.0);
 *   BallPathCalculator finder = new BallPathCalculator(balls, robotPose, 1.0);
 *   Translation2d[] path = finder.getBestPath();
 */
public class BallPathCalculator {

    /** Width of the intake in meters (28 inches). */
    private static final double INTAKE_WIDTH_METERS = 0.711;

    /** Half the intake width — max perpendicular distance to collect a ball. */
    private static final double INTAKE_RADIUS = INTAKE_WIDTH_METERS / 2.0;

    /**
     * Maximum lateral-to-forward distance ratio the robot can achieve.
     * e.g. 1.5 means the robot can shift 1.5 m sideways per 1 m of forward travel.
     * Replaces the old fixed MAX_X_SHIFT_PER_ROW so reachability scales
     * correctly with the actual distance between clusters.
     */
    private static final double MAX_LATERAL_RATIO = 1.5;

    /**
     * Score penalty per meter of lateral shift, expressed in "fraction of a ball."
     * A value of 0.3 means 1 m of lateral shift costs 0.3 balls worth of score,
     * so straight paths win whenever ball counts are tied.
     */
    private static final double LATERAL_COST_PER_METER = 0.3;

    /**
     * Maximum Y gap between two balls before they are split into separate clusters.
     */
    private static final double CLUSTER_EPSILON_METERS = 0.3;

    /** Spread around each ball's X for candidate position generation. */
    private static final double CANDIDATE_X_SPREAD = INTAKE_RADIUS;

    private final Translation2d[] ballPoses;
    private final Translation2d robotStart;
    private final double sweepDirection;
    private final double sweepYLimit;

    public BallPathCalculator(Translation2d[] ballPoses, Translation2d robotStart,
                              double sweepDirection) {
        this(ballPoses, robotStart, sweepDirection,
             Math.signum(sweepDirection) > 0
                 ? Double.POSITIVE_INFINITY
                 : Double.NEGATIVE_INFINITY);
    }

    public BallPathCalculator(Translation2d[] ballPoses, Translation2d robotStart,
                              double sweepDirection, double sweepYLimit) {
        this.ballPoses      = ballPoses;
        this.robotStart     = robotStart;
        this.sweepDirection = Math.signum(sweepDirection);
        this.sweepYLimit    = sweepYLimit;
    }

    /**
     * Calculates and returns the optimal path as an array of waypoints.
     * One waypoint per ball cluster, progressing in the sweep direction.
     */
    public Translation2d[] getBestPath() {
        if (ballPoses == null || ballPoses.length == 0) {
            return new Translation2d[0];
        }

        List<Translation2d> ahead = filterBallsAhead();
        if (ahead.isEmpty()) return new Translation2d[0];

        List<Cluster> clusters = buildClusters(ahead);
        if (clusters.isEmpty()) return new Translation2d[0];

        double[] candidateXs = buildCandidateXs(ahead);
        if (candidateXs.length == 0) return new Translation2d[0];

        int nClusters = clusters.size();
        int nXs       = candidateXs.length;
        final double UNREACHABLE = Double.NEGATIVE_INFINITY;

        double[][] dp     = new double[nClusters][nXs];
        int[][]    parent = new int[nClusters][nXs];
        for (double[] row : dp)     Arrays.fill(row, UNREACHABLE);
        for (int[]    row : parent) Arrays.fill(row, -1);

        // --- Row 0: approach from robot start to first cluster ---
        double firstY      = clusters.get(0).meanY;
        double approachFwd = Math.abs(firstY - robotStart.getY());
        double maxLat0     = Math.max(approachFwd * MAX_LATERAL_RATIO, INTAKE_RADIUS);

        for (int xi = 0; xi < nXs; xi++) {
            double shift = Math.abs(candidateXs[xi] - robotStart.getX());
            if (shift > maxLat0) continue;

            double balls = countBallsAlongSegment(
                    robotStart.getX(), robotStart.getY(),
                    candidateXs[xi],   firstY);
            dp[0][xi] = balls - LATERAL_COST_PER_METER * shift;
        }

        // --- Subsequent clusters ---
        for (int ci = 1; ci < nClusters; ci++) {
            double curY  = clusters.get(ci).meanY;
            double prevY = clusters.get(ci - 1).meanY;
            double fwd   = Math.abs(curY - prevY);
            double maxLat = Math.max(fwd * MAX_LATERAL_RATIO, INTAKE_RADIUS);

            for (int xi = 0; xi < nXs; xi++) {
                for (int pxi = 0; pxi < nXs; pxi++) {
                    if (dp[ci - 1][pxi] == UNREACHABLE) continue;

                    double shift = Math.abs(candidateXs[xi] - candidateXs[pxi]);
                    if (shift > maxLat) continue;

                    double balls = countBallsAlongSegment(
                            candidateXs[pxi], prevY,
                            candidateXs[xi],  curY);
                    double score = dp[ci - 1][pxi] + balls
                                 - LATERAL_COST_PER_METER * shift;

                    if (score > dp[ci][xi]) {
                        dp[ci][xi]     = score;
                        parent[ci][xi] = pxi;
                    }
                }
            }
        }

        // --- Best final state ---
        int bestXi = -1;
        for (int xi = 0; xi < nXs; xi++) {
            if (dp[nClusters - 1][xi] != UNREACHABLE
                && (bestXi == -1 || dp[nClusters - 1][xi] > dp[nClusters - 1][bestXi])) {
                bestXi = xi;
            }
        }
        if (bestXi == -1) return new Translation2d[0];

        // --- Trace back ---
        List<Translation2d> path = new ArrayList<>();
        int xi = bestXi;
        for (int ci = nClusters - 1; ci >= 0; ci--) {
            path.add(0, new Translation2d(candidateXs[xi], clusters.get(ci).meanY));
            if (ci > 0 && parent[ci][xi] >= 0) {
                xi = parent[ci][xi];
            }
        }

        return simplifyPath(path).toArray(new Translation2d[0]);
    }

    /**
     * Returns the estimated number of balls the best path will collect,
     * including the approach segment from the robot's starting position.
     */
    public int getEstimatedBallCount() {
        Translation2d[] path = getBestPath();
        if (path.length == 0) return 0;

        int count = 0;
        List<Translation2d> collected = new ArrayList<>();
        Translation2d prev = robotStart;

        for (Translation2d wp : path) {
            for (Translation2d ball : ballPoses) {
                if (!collected.contains(ball)
                    && distanceToSegment(ball, prev, wp) <= INTAKE_RADIUS) {
                    collected.add(ball);
                    count++;
                }
            }
            prev = wp;
        }
        return count;
    }

    // ---------------------------------------------------------------------
    // Internal types
    // ---------------------------------------------------------------------

    private static class Cluster {
        final double meanY;
        final List<Translation2d> balls;

        Cluster(double meanY, List<Translation2d> balls) {
            this.meanY = meanY;
            this.balls  = balls;
        }
    }

    // ---------------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------------

    private List<Translation2d> filterBallsAhead() {
        List<Translation2d> result = new ArrayList<>();
        double robotY = robotStart.getY();
        for (Translation2d ball : ballPoses) {
            double by = ball.getY();
            if (sweepDirection > 0 && by < robotY)      continue;
            if (sweepDirection < 0 && by > robotY)      continue;
            if (sweepDirection > 0 && by > sweepYLimit)  continue;
            if (sweepDirection < 0 && by < sweepYLimit)  continue;
            result.add(ball);
        }
        return result;
    }

    /**
     * Groups balls into clusters by Y proximity (sorted in travel order),
     * then returns one cluster per group with the mean Y as its anchor.
     */
    private List<Cluster> buildClusters(List<Translation2d> balls) {
        List<Translation2d> sorted = new ArrayList<>(balls);
        if (sweepDirection > 0) {
            sorted.sort((a, b) -> Double.compare(a.getY(), b.getY()));
        } else {
            sorted.sort((a, b) -> Double.compare(b.getY(), a.getY()));
        }

        List<Cluster> clusters = new ArrayList<>();
        List<Translation2d> current = new ArrayList<>();
        current.add(sorted.get(0));

        for (int i = 1; i < sorted.size(); i++) {
            if (Math.abs(sorted.get(i).getY() - sorted.get(i - 1).getY())
                    <= CLUSTER_EPSILON_METERS) {
                current.add(sorted.get(i));
            } else {
                clusters.add(finishCluster(current));
                current = new ArrayList<>();
                current.add(sorted.get(i));
            }
        }
        clusters.add(finishCluster(current));
        return clusters;
    }

    private static Cluster finishCluster(List<Translation2d> balls) {
        double sum = 0;
        for (Translation2d b : balls) sum += b.getY();
        return new Cluster(sum / balls.size(), new ArrayList<>(balls));
    }

    private double[] buildCandidateXs(List<Translation2d> balls) {
        List<Double> candidates = new ArrayList<>();
        addIfNew(candidates, robotStart.getX());

        for (Translation2d ball : balls) {
            double x = ball.getX();
            addIfNew(candidates, x);
            addIfNew(candidates, x + CANDIDATE_X_SPREAD);
            addIfNew(candidates, x - CANDIDATE_X_SPREAD);
        }

        candidates.sort(Double::compareTo);
        double[] arr = new double[candidates.size()];
        for (int i = 0; i < arr.length; i++) arr[i] = candidates.get(i);
        return arr;
    }

    private static void addIfNew(List<Double> list, double value) {
        for (double existing : list) {
            if (Math.abs(existing - value) < 0.02) return;
        }
        list.add(value);
    }

    private double countBallsAlongSegment(double x1, double y1,
                                          double x2, double y2) {
        int count = 0;
        Translation2d start = new Translation2d(x1, y1);
        Translation2d end   = new Translation2d(x2, y2);
        for (Translation2d ball : ballPoses) {
            if (distanceToSegment(ball, start, end) <= INTAKE_RADIUS) {
                count++;
            }
        }
        return count;
    }

    private static double distanceToSegment(Translation2d point,
                                            Translation2d segStart,
                                            Translation2d segEnd) {
        double dx = segEnd.getX() - segStart.getX();
        double dy = segEnd.getY() - segStart.getY();
        double lengthSq = dx * dx + dy * dy;

        if (lengthSq < 1e-9) return point.getDistance(segStart);

        double t = ((point.getX() - segStart.getX()) * dx
                  + (point.getY() - segStart.getY()) * dy) / lengthSq;
        t = Math.max(0, Math.min(1, t));

        return point.getDistance(new Translation2d(
                segStart.getX() + t * dx,
                segStart.getY() + t * dy));
    }

    /**
     * Removes collinear waypoints — keeps a point only if it deviates
     * meaningfully from the straight line between its neighbours.
     */
    private List<Translation2d> simplifyPath(List<Translation2d> path) {
        if (path.size() <= 2) return path;

        List<Translation2d> simplified = new ArrayList<>();
        simplified.add(path.get(0));

        for (int i = 1; i < path.size() - 1; i++) {
            Translation2d prev = simplified.get(simplified.size() - 1);
            Translation2d curr = path.get(i);
            Translation2d next = path.get(i + 1);

            if (distanceToSegment(curr, prev, next) > 0.05) {
                simplified.add(curr);
            }
        }

        simplified.add(path.get(path.size() - 1));
        return simplified;
    }
}
