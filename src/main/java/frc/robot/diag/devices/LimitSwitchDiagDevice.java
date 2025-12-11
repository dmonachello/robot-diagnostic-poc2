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

    public LimitSwitchDiagDevice(String diagName, int dioChannel) {
        super(diagName);
        this.dioChannel = dioChannel;
    }

    // -------------------- internal helper --------------------

    private void ensureInput() {
        if (input == null) {
            input = new DigitalInput(dioChannel);
        }
    }

    // -------------------- Diag device side --------------------

    @Override
    protected int openHardware() {
        try {
            closeHardware();
            ensureInput();
            return LimitSwitchDiagStatus.S_INIT_OK;
        } catch (Exception e) {
            input = null;
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
    }

    @Override
    protected int runHardwareTest() {
        try {
            ensureInput();
        } catch (Exception e) {
            input = null;
            return LimitSwitchDiagStatus.S_READ_FAULT;
        }

        // Not pressed vs pressed is NOT an error; both are healthy reads.
        try {
            input.get();
            return LimitSwitchDiagStatus.S_READ_OK;
        } catch (Exception e) {
            return LimitSwitchDiagStatus.S_READ_FAULT;
        }
    }

    @Override
    protected void stopHardware() {
        // Nothing to stop for a DIO input
    }

    // -------------------- Terminator side (via TerminatorDeviceBase) --------------------

    @Override
    public String getTerminatorName() {
        return getDiagName();
    }

    @Override
    protected int evalTerminatorStatus() {
        try {
            ensureInput();
        } catch (Exception e) {
            // If we can't even create/read the DIO, don't spuriously kill tests.
            return 0;
        }

        // Adjust polarity if your wiring is opposite
        boolean pressed = !input.get();
        if (pressed) {
            return DiagStatus32.TERM_TEST_TERMINATED_OK;
        }
        return 0;
    }
}
