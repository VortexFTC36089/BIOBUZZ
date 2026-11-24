package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Live gamepad tuner for the custom outtake Ks + Kv + Kp velocity controller.
 *
 * <p>Gamepad 1: A run/stop, LB/RB select wheel, D-pad Left/Right select Ks/Kv/Kp,
 * D-pad Up/Down adjust the selected gain, B/X increase/decrease its step size by
 * a factor of ten. Each gain retains its own step size. Gamepad 2 D-pad Up/Down
 * changes the target RPM, and gamepad 2 A switches between the primary and
 * secondary target RPM presets. Values are also exposed through the configurable
 * panel.
 */
@TeleOp(name = "Test: Outtake Ks Kv Kp Tuning", group = RobotConstants.OpModeGroups.TEST)
public class OuttakeTuning extends OpMode {

    private static final double[] INITIAL_GAIN_STEPS = {0.01, 0.0001, 0.0001};
    private static final double[] MIN_GAIN_STEPS = {0.0001, 0.000001, 0.000001};
    private static final double[] MAX_GAIN_STEPS = {1.0, 0.1, 0.1};
    private static final double TARGET_RPM_STEP = 100.0;
    private static final String[] GAIN_NAMES = {"Ks (V)", "Kv (V/RPM)", "Kp (V/RPM)"};

    private final ToggleButton runToggle = new ToggleButton();
    private final ToggleButton previousWheel = new ToggleButton();
    private final ToggleButton nextWheel = new ToggleButton();
    private final ToggleButton previousGain = new ToggleButton();
    private final ToggleButton nextGain = new ToggleButton();
    private final ToggleButton increaseGain = new ToggleButton();
    private final ToggleButton decreaseGain = new ToggleButton();
    private final ToggleButton increaseGainStep = new ToggleButton();
    private final ToggleButton decreaseGainStep = new ToggleButton();
    private final ToggleButton increaseTarget = new ToggleButton();
    private final ToggleButton decreaseTarget = new ToggleButton();
    private final ToggleButton switchTargetPreset = new ToggleButton();

    private OuttakeSubsystem outtake;
    private final double[] gainSteps = INITIAL_GAIN_STEPS.clone();
    private int wheelIndex;
    private int gainIndex;
    private double targetRpm;
    private boolean running;
    private boolean secondaryTargetSelected;

    @Override
    public void init() {
        outtake = new OuttakeSubsystem(hardwareMap);
        targetRpm = RobotConstants.Outtake.TARGET_RPM;
        outtake.stop();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Outtake Ks/Kv/Kp tuner ready");
        telemetry.addLine("G1 A run/stop | LB/RB wheel | D-pad L/R gain | D-pad U/D adjust");
        telemetry.addLine("G2 D-pad U/D target RPM +/-");
        telemetry.addLine("Start with low target RPM and increase output conservatively.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (previousWheel.update(gamepad1.left_bumper)) {
            wheelIndex = 0;
        }
        if (nextWheel.update(gamepad1.right_bumper)) {
            wheelIndex = 1;
        }
        if (previousGain.update(gamepad1.dpad_left)) {
            gainIndex = (gainIndex + GAIN_NAMES.length - 1) % GAIN_NAMES.length;
        }
        if (nextGain.update(gamepad1.dpad_right)) {
            gainIndex = (gainIndex + 1) % GAIN_NAMES.length;
        }

        if (increaseGain.update(gamepad1.dpad_up)) {
            adjustSelectedGain(1.0);
        }
        if (decreaseGain.update(gamepad1.dpad_down)) {
            adjustSelectedGain(-1.0);
        }
        if (increaseGainStep.update(gamepad1.b)) {
            gainSteps[gainIndex] = Math.min(
                    MAX_GAIN_STEPS[gainIndex], gainSteps[gainIndex] * 10.0);
        }
        if (decreaseGainStep.update(gamepad1.x)) {
            gainSteps[gainIndex] = Math.max(
                    MIN_GAIN_STEPS[gainIndex], gainSteps[gainIndex] / 10.0);
        }
        if (increaseTarget.update(gamepad2.dpad_up)) {
            targetRpm += TARGET_RPM_STEP;
        }
        if (decreaseTarget.update(gamepad2.dpad_down)) {
            targetRpm = Math.max(0.0, targetRpm - TARGET_RPM_STEP);
        }
        if (switchTargetPreset.update(gamepad2.a)) {
            secondaryTargetSelected = !secondaryTargetSelected;
            targetRpm = secondaryTargetSelected
                    ? RobotConstants.Outtake.SECONDARY_TARGET_RPM
                    : RobotConstants.Outtake.TARGET_RPM;
        }
        if (runToggle.update(gamepad1.a)) {
            running = !running;
        }

        if (running) {
            outtake.setTargetRpm(targetRpm);
        } else {
            outtake.stopMotor();
        }
        outtake.update();

        telemetry.addData("Tuner", running ? "RUNNING" : "STOPPED");
        telemetry.addData("Selected wheel", wheelIndex == 0 ? "LEFT" : "RIGHT");
        telemetry.addData("Selected gain", GAIN_NAMES[gainIndex]);
        telemetry.addData("Selected gain step", formatGainStep());
        telemetry.addData("Left Ks / Kv / Kp", "%.3f V / %.6f V/RPM / %.6f V/RPM",
                OuttakeControllerConfig.LEFT_KS_VOLTS,
                OuttakeControllerConfig.LEFT_KV_VOLTS_PER_RPM,
                OuttakeControllerConfig.LEFT_KP_VOLTS_PER_RPM);
        telemetry.addData("Right Ks / Kv / Kp", "%.3f V / %.6f V/RPM / %.6f V/RPM",
                OuttakeControllerConfig.RIGHT_KS_VOLTS,
                OuttakeControllerConfig.RIGHT_KV_VOLTS_PER_RPM,
                OuttakeControllerConfig.RIGHT_KP_VOLTS_PER_RPM);
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Target preset", secondaryTargetSelected ? "SECONDARY" : "PRIMARY");
        telemetry.addData("Measured RPM L/R", "%.0f / %.0f",
                outtake.getLeftRpm(), outtake.getRightRpm());
        telemetry.addData("Error RPM L/R", "%.0f / %.0f",
                outtake.getLeftRpmError(), outtake.getRightRpmError());
        telemetry.addData("Battery voltage", "%.2f V", outtake.getBatteryVoltage());
        telemetry.addData("Command power L/R", "%.3f / %.3f",
                outtake.getLeftPowerCommand(), outtake.getRightPowerCommand());
        telemetry.addData("Motor state", outtake.getMotorState());
        if (!outtake.getControllerFault().isEmpty()) {
            telemetry.addData("Controller fault", outtake.getControllerFault());
        }
        telemetry.update();
    }

    private void adjustSelectedGain(double direction) {
        double value = getSelectedGain() + direction * gainSteps[gainIndex];
        value = Math.max(0.0, value);
        setSelectedGain(value);
    }

    private String formatGainStep() {
        return gainIndex == 0
                ? String.format("%.4f V", gainSteps[gainIndex])
                : String.format("%.6f V/RPM", gainSteps[gainIndex]);
    }

    private double getSelectedGain() {
        if (wheelIndex == 0) {
            switch (gainIndex) {
                case 0:
                    return OuttakeControllerConfig.LEFT_KS_VOLTS;
                case 1:
                    return OuttakeControllerConfig.LEFT_KV_VOLTS_PER_RPM;
                default:
                    return OuttakeControllerConfig.LEFT_KP_VOLTS_PER_RPM;
            }
        }
        switch (gainIndex) {
            case 0:
                return OuttakeControllerConfig.RIGHT_KS_VOLTS;
            case 1:
                return OuttakeControllerConfig.RIGHT_KV_VOLTS_PER_RPM;
            default:
                return OuttakeControllerConfig.RIGHT_KP_VOLTS_PER_RPM;
        }
    }

    private void setSelectedGain(double value) {
        if (wheelIndex == 0) {
            switch (gainIndex) {
                case 0:
                    OuttakeControllerConfig.LEFT_KS_VOLTS = value;
                    break;
                case 1:
                    OuttakeControllerConfig.LEFT_KV_VOLTS_PER_RPM = value;
                    break;
                default:
                    OuttakeControllerConfig.LEFT_KP_VOLTS_PER_RPM = value;
                    break;
            }
        } else {
            switch (gainIndex) {
                case 0:
                    OuttakeControllerConfig.RIGHT_KS_VOLTS = value;
                    break;
                case 1:
                    OuttakeControllerConfig.RIGHT_KV_VOLTS_PER_RPM = value;
                    break;
                default:
                    OuttakeControllerConfig.RIGHT_KP_VOLTS_PER_RPM = value;
                    break;
            }
        }
    }

    @Override
    public void stop() {
        running = false;
        if (outtake != null) {
            outtake.stop();
        }
    }
}
