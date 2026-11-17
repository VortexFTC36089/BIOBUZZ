package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.LongSupplier;

/**
 * Feeds balls into the flywheels for one burst so that every ball meets the wheels at
 * the same speed and flies the same arc, as fast as the wheels allow.
 *
 * <h3>The idea</h3>
 * Every ball that goes through slows the flywheels. If the next ball arrives before the
 * wheels are back up to speed, it leaves slower and lands short. So:
 * <ol>
 *   <li>When the burst starts, the speed of each wheel is remembered (the "start speed").</li>
 *   <li>The intake and transfer run ONLY while both wheels are within
 *       {@link RobotConstants.ShotFeed#RPM_TOLERANCE} of that start speed.</li>
 *   <li>When a ball pulls a wheel out of that band, the feed pauses (the transfer brakes
 *       and the next ball waits). As soon as the wheels have recovered, it resumes.</li>
 * </ol>
 * With heavy flywheels the wheels never leave the band and the feed never pauses: full
 * rapid fire. With light flywheels it pauses for a moment between balls, which is still
 * the fastest fire that keeps the arc the same.
 *
 * <h3>It can never block a shot</h3>
 * <ul>
 *   <li>If the wheels do not recover within {@link RobotConstants.ShotFeed#MAX_PAUSE_MS},
 *       the feeder stops waiting and feeds the rest of the burst without pausing.</li>
 *   <li>The whole burst ends after {@link RobotConstants.ShotFeed#MAX_TOTAL_MS} at the
 *       latest.</li>
 *   <li>{@link RobotConstants.ShotFeed#GATING_ENABLED} = false switches all of this off
 *       and feeds without pausing for {@link RobotConstants.Auto#FEED_TIME_MS}.</li>
 * </ul>
 *
 * <h3>How to use it</h3>
 * Call {@link #begin()} once when the gate is open, then {@link #step()} once per loop
 * until {@link #isFinished()} is true. Call {@link #end()} if the burst is cut short.
 * {@code intake.update()} and {@code outtake.update()} must still run every loop.
 */
public class ShotFeeder {

    /**
     * After a pause, the wheels must be back inside the band for this many loops in a row
     * before feeding resumes, so one lucky reading on the way up does not restart the feed.
     */
    private static final int RESUME_LOOPS = 2;

    private final IntakeSubsystem intake;
    private final OuttakeSubsystem outtake;
    private final LongSupplier nanoClock;

    // ---- State of the burst in progress ----
    private boolean active;       // a burst is running
    private boolean feeding;      // intake + transfer are commanded on right now
    private boolean gaveUp;       // stopped waiting for the wheels; feeding without pauses
    private boolean notAtSpeedAtStart; // the burst started with the wheels not at speed
    private int inBandStreak;     // loops in a row with both wheels inside the band
    private long burstStartNanos;
    private long lastStepNanos;
    private long pauseStartNanos;
    private double fedMs;         // time the feed has actually been running

    // ---- Numbers from the current / most recent burst, for telemetry ----
    private boolean hasStats;
    private double startLeftRpm;
    private double startRightRpm;
    private double lowestLeftRpm;
    private double lowestRightRpm;
    private int pauseCount;
    private double pausedMs;
    private double longestPauseMs;
    private double burstMs;
    private double transferRpmSum;
    private int transferRpmSamples;

    public ShotFeeder(IntakeSubsystem intake, OuttakeSubsystem outtake) {
        this(intake, outtake, System::nanoTime);
    }

    /** For tests: same as the public constructor, with a clock that can be faked. */
    ShotFeeder(IntakeSubsystem intake, OuttakeSubsystem outtake, LongSupplier nanoClock) {
        this.intake = intake;
        this.outtake = outtake;
        this.nanoClock = nanoClock;
    }

    /**
     * @return {@code true} when the flywheels are ready for a burst to start. With gating
     *         on, both wheels must have been at speed for
     *         {@link RobotConstants.ShotFeed#READY_LOOPS} loops in a row; with gating off
     *         this is the plain {@link OuttakeSubsystem#atSpeed()} check.
     */
    public boolean readyToStart() {
        return RobotConstants.ShotFeed.GATING_ENABLED ? outtake.isSteadyAtSpeed() : outtake.atSpeed();
    }

    /**
     * Starts a burst: remembers the wheel speeds and starts feeding (if the wheels are in
     * the band right now). Call once, after the gate has opened.
     */
    public void begin() {
        long now = nanoClock.getAsLong();
        active = true;
        feeding = false;
        // If the wheels are not even at speed (the spin-up wait timed out), there is no
        // steady speed to hold the balls to. Feed without pauses, exactly as before.
        notAtSpeedAtStart = !outtake.atSpeed();
        gaveUp = notAtSpeedAtStart;
        burstStartNanos = now;
        lastStepNanos = now;
        pauseStartNanos = now;
        fedMs = 0.0;

        // The speed the first ball will meet. Averaged over the last few loops, because
        // the hub reports flywheel speed in coarse steps.
        startLeftRpm = outtake.getRecentLeftRpm();
        startRightRpm = outtake.getRecentRightRpm();
        lowestLeftRpm = outtake.getLeftRpm();
        lowestRightRpm = outtake.getRightRpm();

        pauseCount = 0;
        pausedMs = 0.0;
        longestPauseMs = 0.0;
        burstMs = 0.0;
        transferRpmSum = 0.0;
        transferRpmSamples = 0;
        hasStats = true;

        // Readiness was already checked over several loops, so start right away if the
        // wheels are in the band now.
        inBandStreak = wheelsInBand() ? RESUME_LOOPS : 0;
        applyFeedDecision(now);
    }

    /**
     * Runs one loop of the burst: feed while the wheels are in the band, pause while they
     * are not, and finish when the balls have been fed. Call once per loop after
     * {@link #begin()}; it does nothing when no burst is running.
     */
    public void step() {
        if (!active) {
            return;
        }
        long now = nanoClock.getAsLong();
        double dtMs = (now - lastStepNanos) / 1.0e6;
        lastStepNanos = now;

        if (feeding) {
            // Only time with the feed running counts toward the feed time.
            fedMs += dtMs;
            transferRpmSum += intake.readTransferRpm();
            transferRpmSamples++;
        }
        lowestLeftRpm = Math.min(lowestLeftRpm, outtake.getLeftRpm());
        lowestRightRpm = Math.min(lowestRightRpm, outtake.getRightRpm());

        double totalMs = (now - burstStartNanos) / 1.0e6;
        if (fedMs >= RobotConstants.Auto.FEED_TIME_MS
                || totalMs >= RobotConstants.ShotFeed.MAX_TOTAL_MS) {
            finish(now);
            return;
        }

        inBandStreak = wheelsInBand() ? inBandStreak + 1 : 0;
        applyFeedDecision(now);
    }

    /** @return {@code true} when no burst is running (not started yet, or finished) */
    public boolean isFinished() {
        return !active;
    }

    /**
     * Ends the burst now if one is running: stops the feed it started and switches the
     * flywheel boost off. Safe to call at any time, including when no burst is running.
     */
    public void end() {
        if (active) {
            finish(nanoClock.getAsLong());
        }
    }

    /** Decides whether the feed runs this loop, and commands the intake and transfer. */
    private void applyFeedDecision(long now) {
        boolean allowed;
        if (!RobotConstants.ShotFeed.GATING_ENABLED || gaveUp) {
            allowed = true;
        } else if (feeding) {
            // Keep feeding for as long as the wheels stay inside the band.
            allowed = inBandStreak >= 1;
        } else {
            // Paused: resume only once the wheels have settled back inside the band.
            allowed = inBandStreak >= RESUME_LOOPS;
            if (!allowed && msBetween(pauseStartNanos, now) >= RobotConstants.ShotFeed.MAX_PAUSE_MS) {
                // The wheels are not coming back. Never block the shot: stop waiting and
                // feed the rest of this burst without pauses.
                gaveUp = true;
                allowed = true;
            }
        }

        if (allowed) {
            if (!feeding) {
                closePause(now);
            }
            feeding = true;
            intake.feedShot();
            outtake.setFeedBoost(true);
        } else {
            if (feeding) {
                pauseCount++;
                pauseStartNanos = now;
            }
            feeding = false;
            intake.stop();
            outtake.setFeedBoost(false);
        }
    }

    private void finish(long now) {
        if (feeding) {
            intake.stop();
        } else {
            closePause(now);
        }
        feeding = false;
        active = false;
        outtake.setFeedBoost(false);
        burstMs = (now - burstStartNanos) / 1.0e6;
    }

    /** Adds the pause that just ended to the pause totals. */
    private void closePause(long now) {
        double pauseMs = msBetween(pauseStartNanos, now);
        pausedMs += pauseMs;
        longestPauseMs = Math.max(longestPauseMs, pauseMs);
    }

    /** @return {@code true} if both wheels are within tolerance of their start speed */
    private boolean wheelsInBand() {
        double tolerance = RobotConstants.ShotFeed.RPM_TOLERANCE;
        return Math.abs(outtake.getLeftRpm() - startLeftRpm) <= tolerance
                && Math.abs(outtake.getRightRpm() - startRightRpm) <= tolerance;
    }

    private static double msBetween(long startNanos, long endNanos) {
        return (endNanos - startNanos) / 1.0e6;
    }

    /** @return {@code true} while the intake and transfer are being run by a burst */
    public boolean isFeeding() {
        return feeding;
    }

    /** @return {@code true} if the last burst stopped waiting for the wheels to recover */
    public boolean gaveUpWaiting() {
        return gaveUp;
    }

    /** @return how many times the last burst paused the feed to let the wheels recover */
    public int getPauseCount() {
        return pauseCount;
    }

    /** @return longest single pause (milliseconds) in the last burst: the recovery time */
    public double getLongestPauseMs() {
        return longestPauseMs;
    }

    /** @return time (milliseconds) the feed was actually running in the last burst */
    public double getFedMs() {
        return fedMs;
    }

    /** @return biggest drop (RPM) of the slower wheel below its start speed in the last burst */
    public double getBiggestDipRpm() {
        return Math.max(0.0, Math.max(startLeftRpm - lowestLeftRpm, startRightRpm - lowestRightRpm));
    }

    /**
     * Adds the rapid-fire numbers to the Driver Station. The caller still calls
     * {@code telemetry.update()}. Read these after a burst to tune:
     *
     * <ul>
     *   <li>{@code Burst biggest dip}: how far each wheel dropped below its start speed.
     *       Smaller is better; heavier flywheels make it smaller.</li>
     *   <li>{@code Burst pauses}: how often the feed waited for the wheels, and for how
     *       long. The longest pause is the wheels' recovery time. "GAVE UP" means the
     *       wheels did not recover within the time limit. "NOT AT SPEED AT START" means
     *       the spin-up wait timed out, so the burst was fed without pausing.</li>
     *   <li>{@code Burst feed time}: time the feed ran, inside the total burst time.</li>
     *   <li>{@code Transfer RPM while feeding}: if this is well under the target, the
     *       transfer cannot reach its target and its speed changes with the battery.</li>
     * </ul>
     */
    public void addTelemetry(Telemetry telemetry) {
        String state;
        if (!active) {
            state = "IDLE";
        } else if (feeding) {
            state = gaveUp ? "FEEDING (gave up waiting for the wheels)" : "FEEDING";
        } else {
            state = "WAITING for the wheels to recover";
        }
        if (!RobotConstants.ShotFeed.GATING_ENABLED) {
            state += " (gating OFF)";
        }
        telemetry.addData("Shot feed", state);

        if (!hasStats) {
            telemetry.addData("Burst", "none yet");
            return;
        }
        long now = nanoClock.getAsLong();
        double shownBurstMs = active ? msBetween(burstStartNanos, now) : burstMs;
        telemetry.addData("Burst start RPM (L/R)", "%.0f / %.0f", startLeftRpm, startRightRpm);
        telemetry.addData("Burst biggest dip (L/R)", "%.0f / %.0f RPM",
                Math.max(0.0, startLeftRpm - lowestLeftRpm),
                Math.max(0.0, startRightRpm - lowestRightRpm));
        String note = "";
        if (notAtSpeedAtStart) {
            note = "  WHEELS NOT AT SPEED AT START (no pausing)";
        } else if (gaveUp) {
            note = "  GAVE UP WAITING";
        }
        telemetry.addData("Burst pauses", "%d (longest %.0f ms, total %.0f ms)%s",
                pauseCount, longestPauseMs, pausedMs, note);
        telemetry.addData("Burst feed time", "%.0f ms fed in %.0f ms", fedMs, shownBurstMs);
        if (transferRpmSamples > 0) {
            telemetry.addData("Transfer RPM while feeding", "%.0f (target %.0f)",
                    transferRpmSum / transferRpmSamples, RobotConstants.ShotFeed.TRANSFER_RPM);
        } else {
            telemetry.addData("Transfer RPM while feeding", "no reading yet");
        }
    }
}