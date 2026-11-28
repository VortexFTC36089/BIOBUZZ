package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Works out the turn power that points the whole robot at the hive. It only does math: the
 * OpMode sends the returned power to the follower as its manual turn command.
 *
 * <p>It aims like the turret does: at the top hive side in the top half of the field and at
 * the bottom side in the bottom half, using the same hive coordinates, switch line,
 * hysteresis and pivot offset from {@link RobotConstants.Turret}. The heading it asks for
 * puts the hive straight ahead of the turret pivot, so a turret held at 0 degrees points at
 * it. Gains, tolerances and the aim-point corrections are in {@link RobotConstants.ChassisAlign}.
 *
 * <p>Aim-point calibration: an error that flips direction between the left and right side of
 * the field (left side aims right, right side aims left) cannot come from the heading math or
 * the pivot offset; those give the same error everywhere. It means the point being aimed at
 * is not where the hive really is (usually too close to the robot). Use
 * {@link #estimateHivePoint(Pose, Pose)} to measure the real point, see that method.
 */
public class ChassisAligner {

    private boolean topSide;
    private boolean onTarget;
    private boolean firstUpdate = true;
    private double errorRad;
    private double previousErrorRad;
    private long previousNanos = System.nanoTime();

    /**
     * Call when alignment is switched on.
     *
     * @param startOnTopSide true to start on the top hive side, false for the bottom side
     */
    public void start(boolean startOnTopSide) {
        topSide = startOnTopSide;
        onTarget = false;
        firstUpdate = true;
        errorRad = 0.0;
        previousErrorRad = 0.0;
        previousNanos = System.nanoTime();
    }

    /**
     * Call once per loop while aligning, after follower.update() so the pose is fresh.
     *
     * @param pose current robot pose from the follower
     * @return turn power, positive = counterclockwise; 0 while aligned
     */
    public double turnPower(Pose pose) {
        long now = System.nanoTime();
        double dtSeconds = (now - previousNanos) / 1e9;
        previousNanos = now;
        return turnPower(pose, dtSeconds);
    }

    /** {@link #turnPower(Pose)} with the loop time passed in. */
    double turnPower(Pose pose, double dtSeconds) {
        selectSide(pose.y());

        double error = wrap(headingToHive(pose.x(), pose.y(), topSide) - pose.heading());
        if (Double.isNaN(error)) {
            // A bad pose must not turn the robot.
            onTarget = false;
            firstUpdate = true;
            return 0.0;
        }

        // How fast the error is changing, for damping. Skipped on the first loop, after a
        // side switch and after a stalled loop, where it would be meaningless.
        double errorRate = (!firstUpdate && dtSeconds > 1e-4 && dtSeconds <= 0.5)
                ? wrap(error - previousErrorRad) / dtSeconds : 0.0;
        firstUpdate = false;
        previousErrorRad = error;
        errorRad = error;

        // Aligned, with hysteresis so the robot does not twitch around the target. It only
        // counts as aligned once it has also stopped turning, otherwise it would cut power
        // while still spinning and coast past the target.
        double enter = Math.toRadians(RobotConstants.ChassisAlign.TOLERANCE_DEGREES);
        double exit = Math.max(
                Math.toRadians(RobotConstants.ChassisAlign.EXIT_TOLERANCE_DEGREES), enter);
        double maxRate = Math.toRadians(
                RobotConstants.ChassisAlign.MAX_ALIGNED_RATE_DEGREES_PER_SECOND);
        double absError = Math.abs(error);
        boolean settled = Math.abs(errorRate) <= maxRate;
        onTarget = (absError <= enter && settled) || (onTarget && absError <= exit);
        if (onTarget) {
            return 0.0;
        }

        double power = RobotConstants.ChassisAlign.KP * error
                + RobotConstants.ChassisAlign.KD * errorRate;
        // The friction push only helps outside the tolerance; inside it, it would overshoot.
        if (absError > enter) {
            power += RobotConstants.ChassisAlign.KS * Math.signum(error);
        }
        double max = clip(RobotConstants.ChassisAlign.MAX_TURN_POWER, 0.0, 1.0);
        return clip(power, -max, max);
    }

    /** @return x (inches) of the point aimed at on the given hive side, corrections included */
    public static double aimX(boolean top) {
        return top
                ? RobotConstants.Turret.TOP_TARGET_X_INCHES
                + RobotConstants.ChassisAlign.TOP_AIM_DX_INCHES
                : RobotConstants.Turret.BOTTOM_TARGET_X_INCHES
                + RobotConstants.ChassisAlign.BOTTOM_AIM_DX_INCHES;
    }

    /** @return y (inches) of the point aimed at on the given hive side, corrections included */
    public static double aimY(boolean top) {
        return top
                ? RobotConstants.Turret.TOP_TARGET_Y_INCHES
                + RobotConstants.ChassisAlign.TOP_AIM_DY_INCHES
                : RobotConstants.Turret.BOTTOM_TARGET_Y_INCHES
                + RobotConstants.ChassisAlign.BOTTOM_AIM_DY_INCHES;
    }

    /**
     * @return the field heading (radians) that points a straight-ahead turret at the given
     *         hive side from this position, including
     *         {@link RobotConstants.ChassisAlign#TRIM_DEGREES}
     */
    public static double headingToHive(double x, double y, boolean top) {
        double dx = aimX(top) - x;
        double dy = aimY(top) - y;
        double distance = Math.hypot(dx, dy);
        // The pivot sits OFFSET_Y to the left of the robot's center (Pedro robot frame:
        // +x forward, +y left), so the center has to point that much to the side of the hive.
        // OFFSET_X lies along the shot line and does not change the heading.
        double offsetAngle = distance > 1e-6
                ? Math.asin(clip(RobotConstants.Turret.OFFSET_Y_INCHES / distance, -1.0, 1.0))
                : 0.0;
        return wrap(Math.atan2(dy, dx) - offsetAngle
                + Math.toRadians(RobotConstants.ChassisAlign.TRIM_DEGREES));
    }

    /**
     * Finds where the hive really is from two poses where the robot (turret at 0) was pointed
     * at it by hand. Take one on the left side of the field and one on the right side, at
     * least 30 degrees apart in heading. Each pose gives a line out of the turret pivot; the
     * hive is where the two lines cross.
     *
     * @return {x, y} in inches, or null if the two headings are too close to tell
     */
    public static double[] estimateHivePoint(Pose a, Pose b) {
        double[] pa = pivot(a);
        double[] pb = pivot(b);
        double ax = Math.cos(a.heading());
        double ay = Math.sin(a.heading());
        double bx = Math.cos(b.heading());
        double by = Math.sin(b.heading());
        double cross = ax * by - ay * bx;
        if (Math.abs(cross) < Math.sin(Math.toRadians(10.0))) {
            return null;
        }
        double t = ((pb[0] - pa[0]) * by - (pb[1] - pa[1]) * bx) / cross;
        return new double[] {pa[0] + t * ax, pa[1] + t * ay};
    }

    /**
     * Same as {@link #estimateHivePoint(Pose, Pose)} but for any number of poses (2 or more),
     * using the point closest to all the aim lines. Take them from the spots you actually
     * shoot from, on both sides of the field: with wrong odometry offsets the error changes
     * with heading, so the result is best near where the samples were taken.
     *
     * @return {x, y} in inches, or null if there are fewer than 2 poses or the headings are
     *         all nearly the same
     */
    public static double[] estimateHivePoint(Pose... poses) {
        double a11 = 0, a12 = 0, a22 = 0, b1 = 0, b2 = 0;
        for (Pose p : poses) {
            double[] pv = pivot(p);
            double ux = Math.cos(p.heading());
            double uy = Math.sin(p.heading());
            // (I - u u^T): the part of a offset that is sideways to the aim line.
            double m11 = 1 - ux * ux;
            double m12 = -ux * uy;
            double m22 = 1 - uy * uy;
            a11 += m11;
            a12 += m12;
            a22 += m22;
            b1 += m11 * pv[0] + m12 * pv[1];
            b2 += m12 * pv[0] + m22 * pv[1];
        }
        double det = a11 * a22 - a12 * a12;
        if (poses.length < 2 || Math.abs(det) < 1e-3) {
            return null;
        }
        return new double[] {(a22 * b1 - a12 * b2) / det, (a11 * b2 - a12 * b1) / det};
    }

    /** @return field position of the turret pivot for this robot pose */
    private static double[] pivot(Pose pose) {
        double c = Math.cos(pose.heading());
        double s = Math.sin(pose.heading());
        double ox = RobotConstants.Turret.OFFSET_X_INCHES;
        double oy = RobotConstants.Turret.OFFSET_Y_INCHES;
        return new double[] {pose.x() + c * ox - s * oy, pose.y() + s * ox + c * oy};
    }

    private void selectSide(double y) {
        double mid = RobotConstants.Turret.SIDE_SWITCH_Y_INCHES;
        double band = RobotConstants.Turret.SIDE_HYSTERESIS_INCHES;
        boolean top = topSide;
        if (y > mid + band) {
            top = true;
        } else if (y < mid - band) {
            top = false;
        }
        if (top != topSide) {
            // The target jumps to the other side.
            topSide = top;
            onTarget = false;
            firstUpdate = true;
        }
    }

    /** @return true while aiming at the top hive side */
    public boolean isAimingAtTopSide() {
        return topSide;
    }

    /** @return true while the robot is within tolerance of the aim heading and has stopped */
    public boolean isAligned() {
        return onTarget;
    }

    /** @return heading still to turn, degrees, positive = counterclockwise */
    public double getErrorDegrees() {
        return Math.toDegrees(errorRad);
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
