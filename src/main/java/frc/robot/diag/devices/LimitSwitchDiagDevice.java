package frc.robot.diag.devices;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.diag.core.DiagStatus32;
import frc.robot.diag.core.TerminatorDeviceBase;

/**
 * Limit switch diagnostic device that can also act as a terminator.
 */
public class LimitSwitchDiagDevice extends TerminatorDeviceBase {

    private final int dioChannel;
    private DigitalInput input;

    private int lastRaw = -1;       // 0/1, -1 = unknown
    private int lastPressed = -1;   // 0/1, -1 = unknown

    public LimitSwitchDiagDevice(String diagName, int dioChannel) {
        super(diagName);
        this.dioChannel = dioChannel;
    }

    @Override
    protected int openHardware() {
        try {
            closeHardware();
            input = new DigitalInput(dioChannel);
            return DiagStatus32.S_INIT_OK;
        } catch (Exception e) {
            input = null;
            return DiagStatus32.S_INIT_FAIL;
        }
    }

    @Override
    protected void closeHardware() {
        if (input != null) {
            try {
                input.close();
            } catch (Exception ignored) {}
            input = null;
        }
    }

    @Override
    protected int runHardwareTest() {
        if (input == null) {
            return DiagStatus32.S_HW_NOT_PRESENT;
        }

        try {
            boolean raw = input.get();
            lastRaw = raw ? 1 : 0;
            lastPressed = raw ? 0 : 1;  // typical pull-up wiring (pressed => raw false)

            return DiagStatus32.S_TEST_OK;
        } catch (Exception e) {
            return DiagStatus32.S_HW_FAULT;
        }
    }

    @Override
    protected void stopHardware() {
        // Nothing to stop for a DIO input
    }

    @Override
    protected int evalTerminatorStatus() {

        // This must be an error now.
        // If we are armed and evaluating, the hardware should already be opened.
        if (input == null) {
            return LimitSwitchDiagStatus.S_INPUT_NULL;
        }

        try {
            boolean raw = input.get();
            lastRaw = raw ? 1 : 0;
            lastPressed = raw ? 0 : 1;  // typical pull-up wiring

            boolean pressed = (lastPressed == 1);
            if (pressed) {
                return DiagStatus32.TERM_TEST_TERMINATED_OK;
            }

            return DiagStatus32.TERM_CONTINUE;

        } catch (Exception e) {
            return DiagStatus32.TERM_TEST_TERMINATED_BAD;
        }
    }

    @Override
    public String getTerminatorDebug() {
        return "raw=" + lastRaw + " pressed=" + lastPressed;
    }
}
