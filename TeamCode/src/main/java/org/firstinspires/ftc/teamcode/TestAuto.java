package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "2_Tip_Auto", group = "Autonomous",
        preselectTeleOp = RobotConstants.OpModeNames.TELEOP_AFTER_AUTO)
public class TestAuto extends LinearOpMode {

    /**
     * Set to {@code true} to use the turret in this autonomous again. While {@code false}
     * the turret servos and encoder are never touched, so the turret does not aim: point it
     * straight forward by hand before Play. TeleOp then takes the turret zero at its own
     * Init.
     */
    private static final boolean TURRET_ENABLED = false;

    private Follower follower;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private TurretSubsystem turret;
    private Servo flowerServo;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // The path points. They come from RobotConstants.TestAutoPoints (editable in Panels) and
    // are rebuilt by refreshPoints() while waiting for Play and when Play is pressed, so they
    // are not final. Control points only use X and Y (a Bezier control point has no heading).
    private Pose start;
    private final Pose shootSpot = poseFactory.of(59, 11, 90);
    private final Pose somth = poseFactory.of(23.5, 57, 180);
    private Pose point1;
    private Pose point2;
    private Pose point2Control1;

    private Pose point3;
    private Pose point3Control1;
    private Pose point3Control2;

    private Pose point4;
    private Pose point4Control1;
    private Pose point4Control2;
    private Pose point5;

    private Pose point6;
    private Pose park;

    // The start pose last given to the follower, so an edit of it can be detected.
    private boolean startApplied = false;
    private double appliedStartX;
    private double appliedStartY;
    private double appliedStartHeadingDeg;

    /** Rebuilds every point from the live values in {@link RobotConstants.TestAutoPoints}. */
    private void refreshPoints() {
        start = poseFactory.of(
                RobotConstants.TestAutoPoints.START_X,
                RobotConstants.TestAutoPoints.START_Y,
                RobotConstants.TestAutoPoints.START_HEADING_DEG);
        point1 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT1_X,
                RobotConstants.TestAutoPoints.POINT1_Y,
                RobotConstants.TestAutoPoints.POINT1_HEADING_DEG);
        point2 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT2_X,
                RobotConstants.TestAutoPoints.POINT2_Y,
                RobotConstants.TestAutoPoints.POINT2_HEADING_DEG);
        point2Control1 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT2_CONTROL1_X,
                RobotConstants.TestAutoPoints.POINT2_CONTROL1_Y, 0);
        point3 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT3_X,
                RobotConstants.TestAutoPoints.POINT3_Y,
                RobotConstants.TestAutoPoints.POINT3_HEADING_DEG);
        point3Control1 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT3_CONTROL1_X,
                RobotConstants.TestAutoPoints.POINT3_CONTROL1_Y, 0);
        point3Control2 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT3_CONTROL2_X,
                RobotConstants.TestAutoPoints.POINT3_CONTROL2_Y, 0);
        point4 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT4_X,
                RobotConstants.TestAutoPoints.POINT4_Y,
                RobotConstants.TestAutoPoints.POINT4_HEADING_DEG);
        point4Control1 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT4_CONTROL1_X,
                RobotConstants.TestAutoPoints.POINT4_CONTROL1_Y, 0);
        point4Control2 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT4_CONTROL2_X,
                RobotConstants.TestAutoPoints.POINT4_CONTROL2_Y, 0);
        point5 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT5_X,
                RobotConstants.TestAutoPoints.POINT5_Y,
                RobotConstants.TestAutoPoints.POINT5_HEADING_DEG);
        point6 = poseFactory.of(
                RobotConstants.TestAutoPoints.POINT6_X,
                RobotConstants.TestAutoPoints.POINT6_Y,
                RobotConstants.TestAutoPoints.POINT6_HEADING_DEG);
        park = poseFactory.of(
                RobotConstants.TestAutoPoints.PARK_X,
                RobotConstants.TestAutoPoints.PARK_Y,
                RobotConstants.TestAutoPoints.PARK_HEADING_DEG);
    }

    /**
     * Re-reads the points from Panels and, the first time and whenever the START values
     * were edited, gives the follower (and the TeleOp hand-off) the new start pose. The
     * follower is only touched when the start changed, not on every loop.
     */
    private void syncStartPose() {
        refreshPoints();
        if (!startApplied
                || RobotConstants.TestAutoPoints.START_X != appliedStartX
                || RobotConstants.TestAutoPoints.START_Y != appliedStartY
                || RobotConstants.TestAutoPoints.START_HEADING_DEG != appliedStartHeadingDeg) {
            follower.setPose(start);
            follower.update();
            // Hand-off to TeleOp starts at the auto start pose, in case auto is
            // stopped before it ever moves.
            blackboard.put(RobotBlackboard.LAST_AUTO_POSE, start);
            appliedStartX = RobotConstants.TestAutoPoints.START_X;
            appliedStartY = RobotConstants.TestAutoPoints.START_Y;
            appliedStartHeadingDeg = RobotConstants.TestAutoPoints.START_HEADING_DEG;
            startApplied = true;
        }
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, Paths.line(start, shootSpot).linear(start, shootSpot)),
                shootBalls(),
                runIntakeAndTransfer(),
                follow(follower, Paths.line(start, point1).linear(start, point1)),
                // Slowed. Pedro applies the limit only while it follows this one path.
                follow(follower, Paths.curve(point1, point2Control1, point2).constant(point2)
                        .with(Constants.foresightConfig.maxPathSpeed.at(Range.clip(
                                RobotConstants.TestAutoPoints.POINT2_SPEED, 0.1, 1.0)))),
                waitMs(750),
                follow(follower, Paths.curve(point2, point3Control1, point3Control2, point3).linear(point2, point3)),
                stopIntakeAndTransfer(),
                waitMs(200),
                shootBalls(),
                follow(follower, Paths.curve(point3, point4Control1, point4Control2, point4).linear(point3, point4)),
                flowerPositionB(),
                runIntakeAndTransfer(),
                waitMs(750),
                follow(follower, Paths.line(point4, point5).linear(point4, point5)),
                twitchFlower(2000),
                follow(follower, Paths.line(point5, point4).linear(point5, point4)),
                stopIntakeAndTransfer(),
                waitMs(200),
                flowerPositionA(),
                // Same curve driven backwards, so the control points are in reverse order.
                follow(follower, Paths.curve(point4, point4Control2, point4Control1, point3).linear(point4, point3)),
                shootBalls(),
                follow(follower, Paths.line(point3, park).linear(point3, park))
        );
    }

    /** Moves the flower servo to position A. */
    public Command flowerPositionA() {
        return instant(() -> flowerServo.setPosition(RobotConstants.FlowerRamp.POSITION_A));
    }

    /** Moves the flower servo to position B. */
    public Command flowerPositionB() {
        return instant(() -> flowerServo.setPosition(RobotConstants.FlowerRamp.POSITION_B));
    }

    /**
     * Twitches the flower servo between positions B and C for {@code durationMs}, starting
     * with a move to C, and leaves it at B.
     */
    public Command twitchFlower(double durationMs) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> {
                    long step = (long) (timer.milliseconds()
                            / RobotConstants.FlowerRamp.TWITCH_INTERVAL_MS);
                    flowerServo.setPosition(step % 2 == 0
                            ? RobotConstants.FlowerRamp.POSITION_C
                            : RobotConstants.FlowerRamp.POSITION_B);
                })
                .setDone(() -> timer.milliseconds() >= durationMs)
                .setEnd(endCondition ->
                        flowerServo.setPosition(RobotConstants.FlowerRamp.POSITION_B));
    }

    /** Starts both the intake and transfer motors. */
    public Command runIntakeAndTransfer() {
        return instant(() -> {
            intake.intake();
            intake.transfer();
        });
    }

    /** Stops both the intake and transfer motors. */
    public Command stopIntakeAndTransfer() {
        return instant(() -> intake.stop());
    }

    /**
     * Shoots the balls currently held by the robot:
     * <ol>
     *   <li>spins the flywheels up and waits until both are at speed (or
     *       {@link RobotConstants.Auto#SPIN_UP_TIMEOUT_MS} passes, so a slow
     *       flywheel cannot stall the whole autonomous),</li>
     *   <li>waits until the turret is aimed at the hive (or
     *       {@link RobotConstants.Turret#AIM_WAIT_TIMEOUT_MS} passes),</li>
     *   <li>opens the gate and waits for the servo to move,</li>
     *   <li>runs the intake and transfer inward for
     *       {@link RobotConstants.Auto#FEED_TIME_MS} to feed the balls,</li>
     *   <li>stops the intake, transfer and flywheels and closes the gate.</li>
     * </ol>
     *
     * <p>Use it inside {@link #autoRoutine()}, e.g. after a {@code follow(...)}.
     * The robot does not drive while this runs.
     */
    public Command TrackShootBalls() {
        return sequential(
                instant(() -> outtake.outtake()),
                race(
                        waitUntil(() -> outtake.atSpeed()),
                        waitMs(RobotConstants.Auto.SPIN_UP_TIMEOUT_MS)
                ),
                race(
                        waitUntil(() -> turret == null
                                || turret.isAimedAtTarget() || turret.hasEncoderFault()),
                        waitMs(RobotConstants.Turret.AIM_WAIT_TIMEOUT_MS)
                ),
                instant(() -> outtake.setGateOpen(true)),
                waitMs(RobotConstants.Auto.GATE_OPEN_WAIT_MS),
                instant(() -> {
                    intake.intake();
                    intake.transfer();
                }),
                waitMs(RobotConstants.Auto.FEED_TIME_MS),
                instant(() -> {
                    intake.stop();
                    outtake.stop();
                })
        );
    }
    public Command shootBalls() {
        return sequential(
                instant(() -> outtake.outtakeFrom(follower.pose())),
                race(
                        waitUntil(() -> outtake.atSpeed()),
                        waitMs(RobotConstants.Auto.SPIN_UP_TIMEOUT_MS)
                ),
                instant(() -> outtake.setGateOpen(true)),
                waitMs(RobotConstants.Auto.GATE_OPEN_WAIT_MS),
                instant(() -> {
                    intake.intake();
                    intake.transfer();
                }),
                waitMs(RobotConstants.Auto.FEED_TIME_MS),
                instant(() -> {
                    intake.stop();
                    outtake.stop();
                }),
                instant(() -> outtake.setGateOpen(false))
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        flowerServo = hardwareMap.get(Servo.class, RobotConstants.FlowerRamp.NAME);
        flowerServo.setPosition(RobotConstants.FlowerRamp.POSITION_A);
        follower = Constants.create(hardwareMap);
        // Builds the points from Panels and gives the follower the start pose (and records
        // it as the TeleOp hand-off pose in case auto is stopped before it ever moves).
        syncStartPose();

        // Turret: built AFTER the follower, because its encoder is read through a
        // drive motor port (Turret.ENCODER_PORT). Not built at all while disabled.
        if (TURRET_ENABLED) {
            turret = new TurretSubsystem(hardwareMap);
            turret.setMode(TurretSubsystem.TurretMode.TRACKING);
        }

        // While waiting for Play, keep declaring "where the turret points now" as
        // straight forward, so the zero is the one at the moment Play is pressed.
        // The turret must be facing the same way as the robot then. TeleOp restores
        // this zero, so it knows where the turret really points after autonomous.
        while (opModeInInit()) {
            int turretZeroTicks = 0;
            if (turret != null) {
                turret.resetAngle();
                turretZeroTicks = turret.getZeroOffsetTicks();
            }
            if (isStopRequested()) {
                // Stop interrupts the hub read and the SDK then returns 0 instead of
                // the real count. Keep the last good count.
                break;
            }
            blackboard.put(RobotBlackboard.LAST_AUTO_TURRET_ZERO_TICKS, turretZeroTicks);
            // Pick up path points edited in Panels (a changed start pose moves the follower).
            syncStartPose();
            // Same for the flower ramp's starting position.
            flowerServo.setPosition(RobotConstants.FlowerRamp.POSITION_A);

            telemetry.addLine("AutoPath Bottom ready");
            telemetry.addLine("Path points: edit them in Panels (RobotConstants > TestAutoPoints)"
                    + " before Play.");
            telemetry.addData("Start X / Y / Heading", "%.1f in / %.1f in / %.1f deg",
                    RobotConstants.TestAutoPoints.START_X,
                    RobotConstants.TestAutoPoints.START_Y,
                    RobotConstants.TestAutoPoints.START_HEADING_DEG);
            if (turret == null) {
                telemetry.addLine("Turret: DISABLED in this auto (TURRET_ENABLED = false)");
            }
            telemetry.addLine("Point the turret STRAIGHT FORWARD (same way as the robot)."
                    + " Its zero is taken when you press Play.");
            telemetry.addData("Turret encoder", "port '%s', ticks %d",
                    RobotConstants.Turret.ENCODER_PORT, turretZeroTicks);
            telemetry.addData("TeleOp after this auto", RobotConstants.OpModeNames.TELEOP_AFTER_AUTO);
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            return;
        }
        // Final read of the Panels values: every path below is built from them right now.
        syncStartPose();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            // Save the latest pose every loop so TeleOp starts from wherever
            // autonomous ended, even if it is stopped early or times out.
            // A Stop can interrupt the Pinpoint read and return invalid values,
            // so do not overwrite the last good pose (and do not aim with it).
            if (!isStopRequested()) {
                blackboard.put(RobotBlackboard.LAST_AUTO_POSE, follower.pose());
                // Aim at the hive; the robot's Y picks the top or bottom side.
                if (turret != null) {
                    turret.update(follower.pose());
                }
            }
            Scheduler.execute();
            // Apply whatever the commands requested this loop to the hardware.
            intake.update();
            outtake.update();

            if (turret != null) {
                turret.addTelemetry(telemetry);
            } else {
                telemetry.addData("Turret", "DISABLED (TURRET_ENABLED = false)");
            }
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Outtake RPM (L/R)", "%.0f / %.0f (atSpeed=%b)",
                    outtake.getLeftRpm(), outtake.getRightRpm(), outtake.atSpeed());
            telemetry.addData("Outtake Gate", outtake.isGateOpen() ? "OPEN" : "CLOSED");
            // The last position sent to the servo, not a measurement of where it is.
            telemetry.addData("Flower servo command", "%.2f", flowerServo.getPosition());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        // Ends any path still running, so a Stop during the slowed curve cannot leave its
        // speed limit on for later OpModes.
        follower.stop();
        flowerServo.setPosition(RobotConstants.FlowerRamp.POSITION_A);
        if (turret != null) {
            turret.stop();
        }
    }
}
