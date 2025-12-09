package frc.robot.diag.devices;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.core.DiagStatus32;
import frc.robot.diag.core.DiagTerminator;

/**
 * Limit switch diagnostic device that can also act as a terminator.
 */
public class LimitSwitchDiagDevice extends DiagDeviceBase implements DiagTerminator {

    private final int dioChannel;
    private DigitalInput input;

    public LimitSwitchDiagDevice(String diagName, int dioChannel) {
        super(diagName);
        this.dioChannel = dioChannel;
    }

    @Override
    protected int openHardware() {
        try {
            closeHardware();
            input = new DigitalInput(dioChannel);
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
        if (input == null) {
            return DiagStatus32.S_HW_NOT_PRESENT;
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

    // ----------------------------------------------------------------
    // DiagTerminator implementation
    // ----------------------------------------------------------------

    @Override
    public int getTerminatorStatus() {
        if (input == null) {
            return 0;
        }

        boolean pressed = !input.get();  // adjust if your wiring is opposite
        if (pressed) {
            return DiagStatus32.TERM_TEST_TERMINATED_OK;
        }
        return 0;
    }

    @Override
    public String getTerminatorName() {
        return getDiagName();
    }
}
