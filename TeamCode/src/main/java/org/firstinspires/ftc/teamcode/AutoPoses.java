package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * Every field position the autonomous drives to, for either alliance.
 *
 * <p>Edit the numbers in {@link #forAlliance(Alliance)} only. Red poses are
 * produced by mirroring blue across the field centre line, so one set of
 * measurements covers both alliances.
 *
 * <p>Units and frame (Pedro Pathing 3 defaults):
 * <ul>
 *   <li>x and y are inches on a 144 in square field, origin at a corner</li>
 *   <li>headings below are in <b>degrees</b>; {@link PoseFactory#degrees()}
 *       converts them to the radians Pedro uses internally</li>
 *   <li>heading 0 points along +x, counter-clockwise is positive</li>
 * </ul>
 *
 * <p>How to measure a pose: run the Pedro "Tests" tuner, pick "Pose Test",
 * push the robot to the spot by hand, and copy the x / y / heading it
 * reports. Do the same for the start pose with the robot against the wall.
 */
public final class AutoPoses {

    private AutoPoses() {
        // Static holder only.
    }

    /**
     * Which side of the field the robot starts on.
     */
    public enum Alliance {
        BLUE,
        RED
    }

    /**
     * Field edge length in inches.
     */
    public static final double FIELD_SIZE_INCHES = 144.0;

    /**
     * How far the robot creeps forward, along its current heading, to
     * sweep balls out of a flower with the intake running. It drives this
     * distance forward and then the same distance back.
     */
    // TODO(team): tune so the intake reaches the balls without hitting the wall
    public static final double FLOWER_PUSH_INCHES = 6.0;

    /**
     * Builds poses from degree headings, unmirrored.
     */
    private static final PoseFactory BLUE_FACTORY = PoseFactory.degrees();

    /**
     * Same numbers reflected across the field centre line (y = 72): y
     * becomes 144 - y and the heading flips sign.
     */
    // TODO(team): confirm red is mirrored top/bottom (y) and not left/right (x)
    private static final PoseFactory RED_FACTORY = PoseFactory.degrees()
            .mapY(y -> FIELD_SIZE_INCHES - y)
            .mapHeading(h -> -h);

    /**
     * The poses one autonomous run drives through, in order.
     */
    public static final class Routine {
        /** Start of the match, which is also shooting position 1. */
        public final Pose shootPosition1;
        /** In front of the lower flower, intake facing it. */
        public final Pose flower1;
        /** Shooting position 2: where the robot shoots at the hive. */
        public final Pose shootPosition2;
        /** In front of the second flower, intake facing it. */
        public final Pose flower2;

        private Routine(Pose shootPosition1, Pose flower1, Pose shootPosition2, Pose flower2) {
            this.shootPosition1 = shootPosition1;
            this.flower1 = flower1;
            this.shootPosition2 = shootPosition2;
            this.flower2 = flower2;
        }
    }

    /**
     * Builds the routine for one alliance.
     *
     * @param alliance BLUE uses the numbers as written, RED mirrors them
     * @return the poses for that alliance
     */
    public static Routine forAlliance(Alliance alliance) {
        PoseFactory f = alliance == Alliance.BLUE ? BLUE_FACTORY : RED_FACTORY;
        return new Routine(
                //   x,     y,    heading (deg)
                f.of(56.0, 8.0, 90.0),     // shooting position 1 (start)
                f.of(11.0, 47.0, 180.0),   // lower flower
                f.of(58.0, 130.0, 270.0),  // shooting position 2 (at the hive)
                f.of(47.0, 128.0, 90.0)    // second flower
        );
    }

    /**
     * A pose a given distance straight ahead of {@code from}, keeping the
     * same heading. Used for the short push into a flower.
     *
     * @param from   where the robot is
     * @param inches how far forward along its heading (negative = back)
     * @return the shifted pose
     */
    public static Pose forwardOf(Pose from, double inches) {
        return new Pose(
                from.x() + inches * Math.cos(from.heading()),
                from.y() + inches * Math.sin(from.heading()),
                from.heading()
        );
    }
}
