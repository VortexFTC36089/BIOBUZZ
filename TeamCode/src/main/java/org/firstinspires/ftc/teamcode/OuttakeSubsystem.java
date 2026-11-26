package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Dual-motor outtake controlled by custom voltage feedforward and proportional
 * velocity feedback, with a manually controlled game-piece gate.
 *
 * <p>Call {@link #update()} once per OpMode loop. The motors run in
 * {@code RUN_WITHOUT_ENCODER}; encoder velocity is measured in ticks/second and
 * converted to RPM using {@link RobotConstants.Outtake#TICKS_PER_REV}. This
 * subsystem does not use the hub's velocity PIDF controller.
 */
public class OuttakeSubsystem {

    public enum MotorState {
        OFF,
        SPINNING_UP,
        AT_SPEED
    }

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo gateServo;
    private final HardwareMap hardwareMap;

    private boolean gateOpen;
    private boolean gateRequestedOpen;
    private MotorState motorState = MotorState.OFF;
    private double targetRpm;
    /** Kp from {@link ShooterRPMTable} for both wheels; 0 uses OuttakeControllerConfig. */
    private double tableKp;
    private double measuredLeftRpm;
    private double measuredRightRpm;
    private double filteredLeftRpm;
    private double filteredRightRpm;
    private boolean velocityFilterInitialized;
    private double batteryVoltage;
    private double leftPowerCommand;
    private double rightPowerCommand;
    private double leftVoltageCommand;
    private double rightVoltageCommand;
    private String controllerFault = "";

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.LEFT_MOTOR);
        rightMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.RIGHT_MOTOR);

        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        leftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        gateServo = hardwareMap.get(Servo.class, RobotConstants.Outtake.GATE_SERVO);
        gateOpen = false;
        gateRequestedOpen = false;
        gateServo.setPosition(RobotConstants.Outtake.GATE_CLOSED_POSITION);
    }

    /**
     * Requests the gate state. The change is applied on the next {@link #update()}.
     */
    public void setGateOpen(boolean open) {
        gateRequestedOpen = open;
    }

    private void applyGate() {
        if (gateRequestedOpen == gateOpen) {
            return;
        }
        gateOpen = gateRequestedOpen;
        gateServo.setPosition(gateOpen
                ? RobotConstants.Outtake.GATE_OPEN_POSITION
                : RobotConstants.Outtake.GATE_CLOSED_POSITION);
    }

    /** Uses the dashboard-configurable default target RPM. */
    public void outtake() {
        setTargetRpm(RobotConstants.Outtake.TARGET_RPM);
    }

    /**
     * Uses the RPM and Kp that {@link ShooterRPMTable} gives for the robot's distance to
     * the hive.
     *
     * @param pose current robot pose from the follower
     */
    public void outtakeFrom(Pose pose) {
        setTargetRpm(ShooterRPMTable.rpmFor(pose));
        tableKp = ShooterRPMTable.kpFor(pose);
    }

    /**
     * Requests a non-negative flywheel speed, using the Kp values in
     * {@link OuttakeControllerConfig}. Negative values stop the wheels;
     * non-finite values are rejected rather than being allowed into the controller.
     */
    public void setTargetRpm(double rpm) {
        if (Double.isNaN(rpm) || Double.isInfinite(rpm)) {
            throw new IllegalArgumentException("Target RPM must be finite");
        }
        targetRpm = Math.max(0.0, rpm);
        tableKp = 0.0;
        motorState = targetRpm > 0.0 ? MotorState.SPINNING_UP : MotorState.OFF;
        if (targetRpm == 0.0) {
            setMotorPowers(0.0, 0.0);
        }
    }

    /** Immediately removes motor power without waiting for another OpMode loop. */
    public void stopMotor() {
        targetRpm = 0.0;
        motorState = MotorState.OFF;
        setMotorPowers(0.0, 0.0);
    }

    /** Immediately stops both flywheels and closes the gate. */
    public void stop() {
        stopMotor();
        gateRequestedOpen = false;
        applyGate();
    }

    /**
     * Samples encoders and battery voltage, updates the custom controller, and
     * writes both motor powers. Call once per OpMode loop.
     */
    public void update() {
        applyGate();

        double ticksPerRev = RobotConstants.Outtake.TICKS_PER_REV;
        double alpha = OuttakeControllerConfig.VELOCITY_FILTER_ALPHA;
        if (!isFinitePositive(ticksPerRev)
                || !isFinitePositive(alpha)
                || alpha > 1.0) {
            failSafe("TICKS_PER_REV must be positive and filter alpha must be in (0, 1]");
            measuredLeftRpm = 0.0;
            measuredRightRpm = 0.0;
            return;
        }

        measuredLeftRpm = ticksPerSecToRpm(leftMotor.getVelocity(), ticksPerRev);
        measuredRightRpm = ticksPerSecToRpm(rightMotor.getVelocity(), ticksPerRev);
        if (!isFinite(measuredLeftRpm) || !isFinite(measuredRightRpm)) {
            failSafe("Encoder velocity is not finite");
            return;
        }

        if (!velocityFilterInitialized) {
            filteredLeftRpm = measuredLeftRpm;
            filteredRightRpm = measuredRightRpm;
            velocityFilterInitialized = true;
        } else {
            filteredLeftRpm = alpha * measuredLeftRpm + (1.0 - alpha) * filteredLeftRpm;
            filteredRightRpm = alpha * measuredRightRpm + (1.0 - alpha) * filteredRightRpm;
        }

        if (targetRpm <= 0.0) {
            controllerFault = "";
            motorState = MotorState.OFF;
            setMotorPowers(0.0, 0.0);
            return;
        }

        if (!validControllerConfiguration()) {
            failSafe("Invalid Ks/Kv/Kp or maximum-output setting");
            return;
        }

        batteryVoltage = readBatteryVoltage();
        if (!isFinitePositive(batteryVoltage)) {
            failSafe("No valid battery voltage reading");
            return;
        }

        double leftVolts = calculateCommandVoltage(
                OuttakeControllerConfig.LEFT_KS_VOLTS,
                OuttakeControllerConfig.LEFT_KV_VOLTS_PER_RPM,
                getLeftKp(),
                filteredLeftRpm);
        double rightVolts = calculateCommandVoltage(
                OuttakeControllerConfig.RIGHT_KS_VOLTS,
                OuttakeControllerConfig.RIGHT_KV_VOLTS_PER_RPM,
                getRightKp(),
                filteredRightRpm);
        if (!isFinite(leftVolts) || !isFinite(rightVolts)) {
            failSafe("Controller voltage calculation is not finite");
            return;
        }

        double maxPower = OuttakeControllerConfig.MAX_OUTPUT;
        leftPowerCommand = clip(leftVolts / batteryVoltage, 0.0, maxPower);
        rightPowerCommand = clip(rightVolts / batteryVoltage, 0.0, maxPower);
        leftVoltageCommand = leftPowerCommand * batteryVoltage;
        rightVoltageCommand = rightPowerCommand * batteryVoltage;
        leftMotor.setPower(leftPowerCommand);
        rightMotor.setPower(rightPowerCommand);

        double tolerance = RobotConstants.Outtake.RPM_TOLERANCE;
        boolean atSpeed = isFinite(tolerance) && tolerance >= 0.0
                && Math.abs(filteredLeftRpm - targetRpm) <= tolerance
                && Math.abs(filteredRightRpm - targetRpm) <= tolerance;
        motorState = atSpeed ? MotorState.AT_SPEED : MotorState.SPINNING_UP;
        controllerFault = "";
    }

    private boolean validControllerConfiguration() {
        return isFiniteNonNegative(OuttakeControllerConfig.LEFT_KS_VOLTS)
                && isFiniteNonNegative(OuttakeControllerConfig.LEFT_KV_VOLTS_PER_RPM)
                && isFiniteNonNegative(OuttakeControllerConfig.LEFT_KP_VOLTS_PER_RPM)
                && isFiniteNonNegative(OuttakeControllerConfig.RIGHT_KS_VOLTS)
                && isFiniteNonNegative(OuttakeControllerConfig.RIGHT_KV_VOLTS_PER_RPM)
                && isFiniteNonNegative(OuttakeControllerConfig.RIGHT_KP_VOLTS_PER_RPM)
                && isFinite(OuttakeControllerConfig.MAX_OUTPUT)
                && OuttakeControllerConfig.MAX_OUTPUT >= 0.0
                && OuttakeControllerConfig.MAX_OUTPUT <= 1.0;
    }

    private double calculateCommandVoltage(double kS, double kV, double kP, double actualRpm) {
        double feedforwardVolts = kS + kV * targetRpm;
        double feedbackVolts = kP * (targetRpm - actualRpm);
        return feedforwardVolts + feedbackVolts;
    }

    private double readBatteryVoltage() {
        double lowestValidVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.getAll(VoltageSensor.class)) {
            double voltage = sensor.getVoltage();
            if (isFinitePositive(voltage) && voltage < lowestValidVoltage) {
                lowestValidVoltage = voltage;
            }
        }
        return lowestValidVoltage == Double.POSITIVE_INFINITY ? 0.0 : lowestValidVoltage;
    }

    private void failSafe(String reason) {
        controllerFault = reason;
        motorState = targetRpm > 0.0 ? MotorState.SPINNING_UP : MotorState.OFF;
        setMotorPowers(0.0, 0.0);
    }

    private void setMotorPowers(double leftPower, double rightPower) {
        leftPowerCommand = leftPower;
        rightPowerCommand = rightPower;
        leftVoltageCommand = leftPower * batteryVoltage;
        rightVoltageCommand = rightPower * batteryVoltage;
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    private static double ticksPerSecToRpm(double ticksPerSec, double ticksPerRev) {
        return ticksPerSec * 60.0 / ticksPerRev;
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static boolean isFinitePositive(double value) {
        return isFinite(value) && value > 0.0;
    }

    private static boolean isFiniteNonNegative(double value) {
        return isFinite(value) && value >= 0.0;
    }

    private static double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** @return the left wheel Kp in use, in volts per RPM of error */
    public double getLeftKp() {
        return tableKp > 0.0 ? tableKp : OuttakeControllerConfig.LEFT_KP_VOLTS_PER_RPM;
    }

    /** @return the right wheel Kp in use, in volts per RPM of error */
    public double getRightKp() {
        return tableKp > 0.0 ? tableKp : OuttakeControllerConfig.RIGHT_KP_VOLTS_PER_RPM;
    }

    public double getLeftRpm() {
        return measuredLeftRpm;
    }

    public double getRightRpm() {
        return measuredRightRpm;
    }

    public double getCurrentRpm() {
        return (measuredLeftRpm + measuredRightRpm) / 2.0;
    }

    public double getLeftRpmError() {
        return targetRpm - measuredLeftRpm;
    }

    public double getRightRpmError() {
        return targetRpm - measuredRightRpm;
    }

    public double getLeftPowerCommand() {
        return leftPowerCommand;
    }

    public double getRightPowerCommand() {
        return rightPowerCommand;
    }

    public double getLeftVoltageCommand() {
        return leftVoltageCommand;
    }

    public double getRightVoltageCommand() {
        return rightVoltageCommand;
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public String getControllerFault() {
        return controllerFault;
    }

    public boolean atSpeed() {
        return motorState == MotorState.AT_SPEED;
    }

    public MotorState getMotorState() {
        return motorState;
    }
}
