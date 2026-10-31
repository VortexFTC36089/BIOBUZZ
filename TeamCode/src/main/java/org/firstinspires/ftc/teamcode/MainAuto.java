package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * Autonomous OpMode built with Pedro Pathing 3 and Ivy (official command library).
 *
 * <p>Coordinates the mecanum drivetrain ({@link Follower}), intake + transfer
 * ({@link IntakeSubsystem}), and velocity-controlled outtake + servo gate
 * ({@link OuttakeSubsystem}):
 * <ol>
 *   <li>Starts at {@link RobotConstants.Field#START_POSE} and drives to
 *       {@link RobotConstants.Field#PICKUP_POSE} while running the intake and transfer.</li>
 *   <li>Drives to {@link RobotConstants.Field#SCORE_POSE} while spinning the
 *       outtake up to {@link RobotConstants.Outtake#TARGET_RPM}.</li>
 *   <li>Waits until the outtake reaches target RPM, opens the servo gate and runs
 *       the transfer to score, then closes the gate and stops the outtake.</li>
 *   <li>Drives to {@link RobotConstants.Field#PARK_POSE}.</li>
 * </ol>
 */
@Autonomous(name = "Main Auto (Ivy)", group = RobotConstants.OpModeGroups.AUTO, preselectTeleOp = "Main TeleOp")
public class MainAuto extends OpMode {

    /**
     * Maximum time (ms) to wait for the outtake to reach target RPM before
     * opening the gate anyway so autonomous never stalls indefinitely.
     */
    private static final int SPINUP_TIMEOUT_MS = 1500;

    /**
     * How long (ms) to keep the servo gate open and transfer running while
     * scoring game pieces.
     */
    private static final int SCORE_DURATION_MS = 800;

    private Follower follower;
    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;

    private final Pose startPose = RobotConstants.Field.START_POSE;
    private final Pose pickupPose = RobotConstants.Field.PICKUP_POSE;
    private final Pose scorePose = RobotConstants.Field.SCORE_POSE;
    private final Pose parkPose = RobotConstants.Field.PARK_POSE;

    // ---- Paths ----

    private Path startToPickup() {
        return line(startPose, pickupPose).linear(startPose, pickupPose);
    }

    private Path pickupToScore() {
        return line(pickupPose, scorePose).linear(pickupPose, scorePose);
    }

    private Path scoreToPark() {
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }

    // ---- Ivy Mechanism Commands ----

    /**
     * Starts both the intake and transfer motors.
     */
    private Command runIntake() {
        return instant(intake::intake);
    }

    /**
     * Stops both the intake and transfer motors.
     */
    private Command stopIntake() {
        return instant(intake::stop);
    }

    /**
     * Spins the outtake motor up to {@link RobotConstants.Outtake#TARGET_RPM},
     * waits until it reaches speed (with a timeout safety net), opens the servo
     * gate while feeding with the intake/transfer, then closes the gate and
     * stops the motors.
     */
    private Command scoreOuttake() {
        return sequential(
                instant(outtake::outtake),
                race(
                        waitUntil(outtake::atSpeed),
                        waitMs(SPINUP_TIMEOUT_MS)
                ),
                instant(outtake::openGate),
                runIntake(),
                waitMs(SCORE_DURATION_MS),
                instant(outtake::closeGate),
                stopIntake(),
                instant(outtake::stopMotor)
        );
    }

    // ---- Autonomous Routine ----

    /**
     * Full autonomous sequence composed with Ivy's {@code sequential()} and
     * {@code parallel()} groups.
     */
    private Command autoRoutine() {
        return sequential(
                // 1. Drive to pickup while intaking and running transfer
                parallel(
                        follow(follower, startToPickup()),
                        runIntake()
                ),
                stopIntake(),

                // 2. Drive to score pose while pre-spinning the outtake to target RPM
                parallel(
                        follow(follower, pickupToScore()),
                        instant(outtake::outtake)
                ),

                // 3. Score with outtake RPM + servo gate
                scoreOuttake(),

                // 4. Park
                follow(follower, scoreToPark())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        telemetry.addLine("Main Auto (Ivy) initialized");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();
    }

    @Override
    public void start() {
        Scheduler.reset();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        intake.update();
        outtake.update();

        Pose pose = follower.pose();
        PoseStorage.lastAutoPose = pose;

        telemetry.addData("X (in)", "%.1f", pose.x());
        telemetry.addData("Y (in)", "%.1f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Intake", intake.getState());
        telemetry.addData("Outtake RPM", "%.0f / %.0f (atSpeed=%b)",
                outtake.getCurrentRpm(), outtake.getTargetRpm(), outtake.atSpeed());
        telemetry.addData("Outtake Gate", outtake.getGateState());
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (intake != null) {
            intake.stop();
            intake.update();
        }
        if (outtake != null) {
            outtake.stop();
            outtake.update();
        }
        if (follower != null) {
            PoseStorage.lastAutoPose = follower.pose();
            follower.manual(0.0, 0.0, 0.0);
            follower.update();
        }
    }
}
