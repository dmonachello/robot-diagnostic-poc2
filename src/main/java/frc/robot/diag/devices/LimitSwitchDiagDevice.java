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

    // Debug sampling (updated whenever polled)
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
            lastRaw = -1;
            lastPressed = -1;
            return LimitSwitchDiagStatus.S_INIT_OK;
        } catch (Exception e) {
            input = null;
            lastRaw = -1;
            lastPressed = -1;
            return LimitSwitchDiagStatus.S_INIT_FAIL;
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
        lastRaw = -1;
        lastPressed = -1;
    }

    @Override
    protected int runHardwareTest() {
        if (input == null) {
            lastRaw = -1;
            lastPressed = -1;
            return DiagStatus32.S_HW_NOT_PRESENT;
        }

        try {
            sampleInput();
            return LimitSwitchDiagStatus.S_READ_OK;
        } catch (Exception e) {
            lastRaw = -1;
            lastPressed = -1;
            return LimitSwitchDiagStatus.S_READ_FAULT;
        }
    }

    @Override
    protected void stopHardware() {
        // Nothing to stop for a DIO input
    }

    // ------------------------------------------------------------
    // Terminator logic
    // ------------------------------------------------------------

    @Override
    protected int evalTerminatorStatus() {
        if (input == null) {
            lastRaw = -1;
            lastPressed = -1;
            return 0;
        }

        sampleInput();

        if (lastPressed == 1) {
            return DiagStatus32.TERM_TEST_TERMINATED_OK;
        }

        return 0;
    }

    @Override
    public String getTerminatorDebug() {
        return "raw=" + lastRaw + " pressed=" + lastPressed;
    }

    private void sampleInput() {
        boolean raw = input.get();     // true = high
        lastRaw = raw ? 1 : 0;
        lastPressed = raw ? 0 : 1;     // typical pull-up wiring
    }
}
