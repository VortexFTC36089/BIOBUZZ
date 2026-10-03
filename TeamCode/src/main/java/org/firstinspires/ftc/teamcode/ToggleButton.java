package org.firstinspires.ftc.teamcode;

/**
 * Rising-edge detector for a gamepad button.
 *
 * <p>Gamepad buttons stay {@code true} for as long as they are held. OpModes
 * that want "press once to toggle" must react only on the frame the button
 * becomes pressed, not on every loop while it is held. This helper stores the
 * previous value so callers can write:
 *
 * <pre>
 * if (intakeToggle.update(gamepad1.a)) {
 *     intake.toggleIntake();
 * }
 * </pre>
 *
 * <p>This class does not talk to hardware. It is safe to construct in
 * {@code init()} and call once per loop with the current button state.
 */
public class ToggleButton {

    private boolean previousPressed;

    /**
     * Creates a detector that starts as "not pressed."
     *
     * <p>If a button is already held when the OpMode starts, the first
     * {@link #update(boolean)} will still fire once. That matches "the driver
     * is holding A as the match begins" better than silently ignoring it.
     */
    public ToggleButton() {
        previousPressed = false;
    }

    /**
     * Updates the detector with this loop's button value.
     *
     * @param currentlyPressed {@code true} if the button is down this loop
     * @return {@code true} only on the loop where the button went from
     *         released to pressed (a rising edge)
     */
    public boolean update(boolean currentlyPressed) {
        boolean justPressed = currentlyPressed && !previousPressed;
        previousPressed = currentlyPressed;
        return justPressed;
    }

    /**
     * Forgets the last button state.
     *
     * <p>Call this if you reuse the same detector after a long pause and do
     * not want a leftover "held" state to suppress the next press.
     */
    public void reset() {
        previousPressed = false;
    }
}
