package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

/**
 * Sends the outtake and transfer tuning numbers to the Graph widget in Panels.
 *
 * <p>Graph plots Panels telemetry lines of the form {@code NAME: number}. Nothing here is
 * added to the Driver Station screen. The values themselves are edited in Panels under
 * {@code Outtake}, {@code Intake} and {@code OuttakeControllerConfig}.
 *
 * <p>All lines in a graph share one Y axis, so in Panels select only values of a similar
 * size at the same time (RPMs together, small gains together).
 */
public final class TuningGraph {

    private TuningGraph() {
        // Utility class.
    }

    /**
     * Sends one sample. Call once per OpMode loop, after the subsystems' {@code update()}.
     *
     * @param outtake the running outtake
     * @param intake  the running intake and transfer, or {@code null} in an OpMode without
     *                them (the measured transfer RPM is then left out)
     */
    public static void send(OuttakeSubsystem outtake, IntakeSubsystem intake) {
        TelemetryManager graph = PanelsTelemetry.INSTANCE.getTelemetry();

        // Flywheels: what was asked for and what the wheels are really doing.
        graph.addData("TARGET_RPM", RobotConstants.Outtake.TARGET_RPM);
        // The target in use right now (the outtake tuner can change it from the gamepad).
        graph.addData("COMMANDED_RPM", outtake.getTargetRpm());
        graph.addData("LEFT_ACTUAL_RPM", outtake.getLeftRpm());
        graph.addData("RIGHT_ACTUAL_RPM", outtake.getRightRpm());

        // Flywheel gains.
        graph.addData("LEFT_KS_VOLTS", OuttakeControllerConfig.LEFT_KS_VOLTS);
        graph.addData("LEFT_KV_VOLTS_PER_RPM", OuttakeControllerConfig.LEFT_KV_VOLTS_PER_RPM);
        graph.addData("LEFT_KP_VOLTS_PER_RPM", OuttakeControllerConfig.LEFT_KP_VOLTS_PER_RPM);
        graph.addData("RIGHT_KS_VOLTS", OuttakeControllerConfig.RIGHT_KS_VOLTS);
        graph.addData("RIGHT_KV_VOLTS_PER_RPM", OuttakeControllerConfig.RIGHT_KV_VOLTS_PER_RPM);
        graph.addData("RIGHT_KP_VOLTS_PER_RPM", OuttakeControllerConfig.RIGHT_KP_VOLTS_PER_RPM);

        // Transfer.
        graph.addData("TRANSFER_TARGET_RPM", RobotConstants.Intake.TRANSFER_TARGET_RPM);
        if (intake != null) {
            graph.addData("TRANSFER_ACTUAL_RPM", intake.getTransferRpm());
        }
        graph.addData("TRANSFER_KP", RobotConstants.Intake.TRANSFER_KP);
        graph.addData("TRANSFER_KI", RobotConstants.Intake.TRANSFER_KI);
        graph.addData("TRANSFER_KD", RobotConstants.Intake.TRANSFER_KD);
        graph.addData("TRANSFER_KF", RobotConstants.Intake.TRANSFER_KF);

        // Panels only; the Driver Station telemetry is updated by the OpMode itself.
        graph.update();
    }
}
