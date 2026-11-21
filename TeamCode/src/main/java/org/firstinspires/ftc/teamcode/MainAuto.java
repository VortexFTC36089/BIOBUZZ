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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = RobotConstants.OpModeNames.AUTO_BOTTOM, group = "Autonomous",
        preselectTeleOp = RobotConstants.OpModeNames.TELEOP_AFTER_AUTO)
public class MainAuto extends LinearOpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;
    private TurretSubsystem turret;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose point1 = poseFactory.of(12.8, 95, 90);
    private final Pose point1Control1 = poseFactory.of(13.6319, 28.4114, 0);
    private final Pose point2 = poseFactory.of(14, 28, 90);
    private final Pose point3 = poseFactory.of(14, 96.254, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                shootBalls(),
                follow(follower, path1())
        );
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
                        waitUntil(() -> turret.isAimedAtTarget() || turret.hasEncoderFault()),
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
                instant(() -> outtake.outtake()),
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
                })
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        // Hand-off to TeleOp starts at the auto start pose, in case auto is
        // stopped before it ever moves.
        blackboard.put(RobotBlackboard.LAST_AUTO_POSE, start);

        // Turret: built AFTER the follower, because its encoder is read through a
        // drive motor port (Turret.ENCODER_PORT). It tracks the hive from Play on.
        turret = new TurretSubsystem(hardwareMap);
        turret.setMode(TurretSubsystem.TurretMode.TRACKING);

        // While waiting for Play, keep declaring "where the turret points now" as
        // straight forward, so the zero is the one at the moment Play is pressed.
        // The turret must be facing the same way as the robot then. TeleOp restores
        // this zero, so it knows where the turret really points after autonomous.
        while (opModeInInit()) {
            turret.resetAngle();
            if (isStopRequested()) {
                // Stop interrupts the hub read and the SDK then returns 0 instead of
                // the real count. Keep the last good count.
                break;
            }
            blackboard.put(RobotBlackboard.LAST_AUTO_TURRET_ZERO_TICKS, turret.getZeroOffsetTicks());

            telemetry.addLine("AutoPath Bottom ready");
            telemetry.addLine("Point the turret STRAIGHT FORWARD (same way as the robot)."
                    + " Its zero is taken when you press Play.");
            telemetry.addData("Turret encoder", "port '%s', ticks %d",
                    RobotConstants.Turret.ENCODER_PORT, turret.getEncoderTicks());
            telemetry.addData("TeleOp after this auto", RobotConstants.OpModeNames.TELEOP_AFTER_AUTO);
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            return;
        }
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
                turret.update(follower.pose());
            }
            Scheduler.execute();
            // Apply whatever the commands requested this loop to the hardware.
            intake.update();
            outtake.update();

            turret.addTelemetry(telemetry);
            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Outtake RPM (L/R)", "%.0f / %.0f (atSpeed=%b)",
                    outtake.getLeftRpm(), outtake.getRightRpm(), outtake.atSpeed());
            telemetry.addData("Outtake Gate", outtake.isGateOpen() ? "OPEN" : "CLOSED");

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
        turret.stop();
    }

    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }
}
