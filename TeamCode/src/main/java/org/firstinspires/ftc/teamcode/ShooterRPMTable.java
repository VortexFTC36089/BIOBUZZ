package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Locale;

/**
 * Picks the flywheel RPM and Kp from the robot's distance to the hive.
 *
 * <p>You tune a few spots on the field: put the robot on a spot, find the RPM and Kp that
 * score, and enter the spot's X, Y, RPM and Kp as a point. The distance from every point to
 * the hive is worked out, and one equation for RPM and one for Kp are fitted through them:
 *
 * <pre>
 * RPM = a * distance^2 + b * distance + c
 * Kp  = a * distance^2 + b * distance + c   (its own a, b and c)
 * </pre>
 *
 * <ul>
 *    <li>1 tuned distance: that value everywhere</li>
 *    <li>2 tuned distances: a straight line (a = 0)</li>
 *    <li>3 or more: a curve (the best fit if there are more than 3)</li>
 * </ul>
 * Distances less than {@value #MIN_SPREAD_INCHES} inches apart count as one distance.
 *
 * <p>Closer than the nearest tuned spot, or farther than the farthest one, an equation is
 * held at its value at that end. It never guesses beyond what was tuned, so tune the nearest
 * and the farthest spots you shoot from. The Kp also never goes below the lowest or above the
 * highest Kp you tuned.
 *
 * <p>A point's RPM and Kp count separately: 0 means "not tuned yet" and leaves that value out
 * of that equation only. The table's one Kp drives BOTH wheels, replacing the left and right
 * Kp in {@link OuttakeControllerConfig}. Those are used only while the table has no Kp, and by
 * code that does not use the table, such as the Ks Kv Kp tuner.
 *
 * <p>Tuning (Panels, {@code ShooterRPMTable}), in Main TeleOp:
 * <ol>
 *    <li>Set {@link #TUNING_MODE} to true. The flywheels now run at {@link #TUNING_RPM} with
 *        {@link #TUNING_KP}.</li>
 *    <li>Drive to a spot and change {@code TUNING_KP} and {@code TUNING_RPM} until the balls
 *        go in. Kp can shift the real wheel speed a little, so after changing it, check the
 *        RPM again.</li>
 *    <li>Put the X and Y shown on the Driver Station, that RPM and that Kp into a point.</li>
 *    <li>Set {@code TUNING_MODE} back to false.</li>
 * </ol>
 *
 * <p>Panels edits are NOT saved to this file and are lost when the Robot Controller app
 * restarts: when the points are right, copy them here.
 */
@Configurable
public final class ShooterRPMTable {

    /**
     * False ignores this table: always {@link RobotConstants.Outtake#TARGET_RPM} and the Kp
     * in {@link OuttakeControllerConfig}.
     */
    public static boolean ENABLED = true;

    /** True runs the flywheels at {@link #TUNING_RPM} and {@link #TUNING_KP} everywhere. */
    public static boolean TUNING_MODE = false;
    public static double TUNING_RPM = 1950;
    /** Volts per RPM of error, for both wheels. 0 uses {@link OuttakeControllerConfig}. */
    public static double TUNING_KP = 0.031;

    // ---- Tuned spots. X and Y are Pedro field inches. ----
    // KP is in volts per RPM of error. An RPM or KP of 0 = not tuned yet (ignored).

    // Point 1: start position, back against the wall.
    public static double POINT1_X = 58;
    public static double POINT1_Y = 12;
    public static double POINT1_RPM = 1995;
    public static double POINT1_KP = 0.0313;

    // Point 2: corner.
    // TODO(team): tune on the robot
    public static double POINT2_X = 14;
    public static double POINT2_Y = 14;
    public static double POINT2_RPM = 0;
    public static double POINT2_KP = 0;

    // Point 3: a spot close to the hive (change X / Y to wherever you tune).
    // TODO(team): tune on the robot
    public static double POINT3_X = 57;
    public static double POINT3_Y = 35;
    public static double POINT3_RPM = 0;
    public static double POINT3_KP = 0;

    // Points 4 and 5: spare.
    public static double POINT4_X = 0;
    public static double POINT4_Y = 0;
    public static double POINT4_RPM = 0;
    public static double POINT4_KP = 0;

    public static double POINT5_X = 0;
    public static double POINT5_Y = 0;
    public static double POINT5_RPM = 0;
    public static double POINT5_KP = 0;

    /** The table never asks for less than this, whatever the equation says. */
    public static double MIN_RPM = 1000;
    /** The table never asks for more than this, whatever the equation says. */
    public static double MAX_RPM = 3000;

    private static final double MIN_SPREAD_INCHES = 4.0;

    private static final int RPM_COLUMN = 2;
    private static final int KP_COLUMN = 3;

    private ShooterRPMTable() {
        // Utility class.
    }

    /** A fitted equation, {@code value = a*d^2 + b*d + c}, where d is inches to the hive. */
    public static final class Equation {
        public final double a;
        public final double b;
        public final double c;
        public final double nearestInches;
        public final double farthestInches;
        public final double lowestTuned;
        public final double highestTuned;
        public final int pointCount;

        private Equation(double a, double b, double c,
                         double nearestInches, double farthestInches,
                         double lowestTuned, double highestTuned, int pointCount) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.nearestInches = nearestInches;
            this.farthestInches = farthestInches;
            this.lowestTuned = lowestTuned;
            this.highestTuned = highestTuned;
            this.pointCount = pointCount;
        }

        /** @return the value at a distance, which is first kept inside the tuned range */
        public double valueAt(double distanceInches) {
            double d = clip(distanceInches, nearestInches, farthestInches);
            return a * d * d + b * d + c;
        }
    }

    /**
     * Distance in inches from a field position to the hive side on that half of the field.
     * Uses the same hive coordinates as the turret, in {@link RobotConstants.Turret}.
     */
    public static double distanceToHive(double x, double y) {
        boolean top = y >= RobotConstants.Turret.SIDE_SWITCH_Y_INCHES;
        double hiveX = top
                ? RobotConstants.Turret.TOP_TARGET_X_INCHES
                : RobotConstants.Turret.BOTTOM_TARGET_X_INCHES;
        double hiveY = top
                ? RobotConstants.Turret.TOP_TARGET_Y_INCHES
                : RobotConstants.Turret.BOTTOM_TARGET_Y_INCHES;
        return Math.hypot(hiveX - x, hiveY - y);
    }

    /** Fits the RPM equation through the tuned points as they are right now. */
    public static Equation rpmEquation() {
        return fitColumn(RPM_COLUMN, RobotConstants.Outtake.TARGET_RPM);
    }

    /** Fits the Kp equation. Its {@code pointCount} is 0 when no point has a Kp. */
    public static Equation kpEquation() {
        return fitColumn(KP_COLUMN, 0.0);
    }

    private static Equation fitColumn(int column, double fallback) {
        double[][] table = {
                {POINT1_X, POINT1_Y, POINT1_RPM, POINT1_KP},
                {POINT2_X, POINT2_Y, POINT2_RPM, POINT2_KP},
                {POINT3_X, POINT3_Y, POINT3_RPM, POINT3_KP},
                {POINT4_X, POINT4_Y, POINT4_RPM, POINT4_KP},
                {POINT5_X, POINT5_Y, POINT5_RPM, POINT5_KP},
        };
        double[] distances = new double[table.length];
        double[] values = new double[table.length];
        int count = 0;
        for (double[] point : table) {
            if (point[column] > 0.0) {
                distances[count] = distanceToHive(point[0], point[1]);
                values[count] = point[column];
                count++;
            }
        }
        return fit(Arrays.copyOf(distances, count), Arrays.copyOf(values, count), fallback);
    }

    /** Least-squares fit of a value against distance: constant, line or quadratic. */
    private static Equation fit(double[] distances, double[] values, double fallback) {
        int n = distances.length;
        if (n == 0) {
            return new Equation(0.0, 0.0, fallback, 0.0, 0.0, fallback, fallback, 0);
        }

        // How many clearly different distances there are decides the shape. Two spots at
        // almost the same distance would otherwise bend the curve wildly.
        double[] sorted = distances.clone();
        Arrays.sort(sorted);
        int differentDistances = 1;
        double groupStart = sorted[0];
        for (double d : sorted) {
            if (d - groupStart >= MIN_SPREAD_INCHES) {
                differentDistances++;
                groupStart = d;
            }
        }
        int degree = Math.min(2, differentDistances - 1);

        double mean = 0.0;
        for (double d : distances) {
            mean += d / n;
        }

        // Sums are taken around the mean distance, which keeps the arithmetic accurate.
        double s2 = 0.0;
        double s3 = 0.0;
        double s4 = 0.0;
        double t0 = 0.0;
        double t1 = 0.0;
        double t2 = 0.0;
        double lowest = values[0];
        double highest = values[0];
        for (int i = 0; i < n; i++) {
            double u = distances[i] - mean;
            s2 += u * u;
            s3 += u * u * u;
            s4 += u * u * u * u;
            t0 += values[i];
            t1 += u * values[i];
            t2 += u * u * values[i];
            lowest = Math.min(lowest, values[i]);
            highest = Math.max(highest, values[i]);
        }

        double a = 0.0;
        double b = 0.0;
        double c;
        if (degree == 2) {
            a = (t2 - s2 * t0 / n - s3 * t1 / s2) / (s4 - s2 * s2 / n - s3 * s3 / s2);
            b = (t1 - s3 * a) / s2;
            c = (t0 - s2 * a) / n;
        } else if (degree == 1) {
            b = t1 / s2;
            c = t0 / n;
        } else {
            c = t0 / n;
        }

        // Convert from "distance minus mean" back to plain distance.
        return new Equation(a, b - 2.0 * a * mean, c - b * mean + a * mean * mean,
                sorted[0], sorted[n - 1], lowest, highest, n);
    }

    /** @return the flywheel RPM to use with the robot at {@code pose} */
    public static double rpmFor(Pose pose) {
        if (!ENABLED) {
            return RobotConstants.Outtake.TARGET_RPM;
        }
        if (TUNING_MODE) {
            return clip(TUNING_RPM, MIN_RPM, MAX_RPM);
        }
        double distance = distanceToHive(pose.x(), pose.y());
        double rpm = clip(rpmEquation().valueAt(distance), MIN_RPM, MAX_RPM);
        // A bad pose must not reach the flywheels.
        return Double.isNaN(rpm) || Double.isInfinite(rpm)
                ? RobotConstants.Outtake.TARGET_RPM : rpm;
    }

    /**
     * @return the Kp for both wheels with the robot at {@code pose}, or 0 when the values in
     *         {@link OuttakeControllerConfig} should be used
     */
    public static double kpFor(Pose pose) {
        if (!ENABLED) {
            return 0.0;
        }
        double kp;
        if (TUNING_MODE) {
            kp = TUNING_KP;
        } else {
            Equation equation = kpEquation();
            double distance = distanceToHive(pose.x(), pose.y());
            kp = clip(equation.valueAt(distance), equation.lowestTuned, equation.highestTuned);
        }
        // Only a usable gain may reach the controller (NaN fails this comparison too).
        return kp > 0.0 && !Double.isInfinite(kp) ? kp : 0.0;
    }

    /** Distance, RPM, Kp and equations for the Driver Station. */
    public static void addTelemetry(Telemetry telemetry, Pose pose) {
        String source = !ENABLED ? "fixed TARGET_RPM"
                : TUNING_MODE ? "TUNING_RPM" : "from equation";
        telemetry.addData("Hive distance (in)", "%.1f", distanceToHive(pose.x(), pose.y()));
        telemetry.addData("Shooter RPM", "%.0f (%s)", rpmFor(pose), source);

        double kp = kpFor(pose);
        if (kp > 0.0) {
            telemetry.addData("Shooter Kp", "%.5f (%s)", kp,
                    TUNING_MODE ? "TUNING_KP" : "from equation");
        } else {
            telemetry.addData("Shooter Kp", "controller config, L %.5f / R %.5f",
                    OuttakeControllerConfig.LEFT_KP_VOLTS_PER_RPM,
                    OuttakeControllerConfig.RIGHT_KP_VOLTS_PER_RPM);
        }

        telemetry.addData("RPM equation", describe(rpmEquation(), "%.4f*d^2 %+.3f*d %+.1f"));
        telemetry.addData("Kp equation", describe(kpEquation(), "%.8f*d^2 %+.6f*d %+.5f"));
    }

    private static String describe(Equation e, String format) {
        if (e.pointCount == 0) {
            return "no tuned points";
        }
        return String.format(Locale.US, format + "  (%d points, d = %.0f to %.0f in)",
                e.a, e.b, e.c, e.pointCount, e.nearestInches, e.farthestInches);
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
