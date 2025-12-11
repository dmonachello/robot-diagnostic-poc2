package frc.robot.diag.devices;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.diag.core.DiagStatus32;
import frc.robot.diag.core.TerminatorDeviceBase;

/**
 * Pure time-based terminator that can also show up as a diag device.
 *
 * As a terminator:
 *  - On each DUT test start, the timer is reset.
 *  - Once elapsed time exceeds durationSeconds, it returns
 *    TERM_TEST_TERMINATED_OK (one or more times).
 */
public class TimerTerminator extends TerminatorDeviceBase {

    private final double durationSeconds;
    private double startTimeSec = -1.0;

    public TimerTerminator(String diagName, double durationSeconds) {
        super(diagName);
        this.durationSeconds = durationSeconds;
    }

    // -------------------- Diag device side --------------------

    @Override
    protected int openHardware() {
        // Nothing to open; this is purely time-based.
        startTimeSec = -1.0;
        return DiagStatus32.S_INIT_OK;
    }

    @Override
    protected void closeHardware() {
        // Nothing to close
    }

    @Override
    protected int runHardwareTest() {
        // As a first-class diag device, just report "test ok"
        return DiagStatus32.S_TEST_OK;
    }

    @Override
    protected void stopHardware() {
        // Nothing to stop
    }

    // -------------------- Terminator lifecycle --------------------

    @Override
    public void onTestStart() {
        super.onTestStart();
        startTimeSec = -1.0;  // will be set on first evalTerminatorStatus()
    }

    @Override
    public void onTestEnd() {
        super.onTestEnd();
        // leave startTimeSec; it will be reset next test
    }

    @Override
    public String getTerminatorName() {
        return getDiagName();
    }

    @Override
    protected int evalTerminatorStatus() {
        if (!termActive) {
            return 0;
        }

        double now = Timer.getFPGATimestamp();
        if (startTimeSec < 0.0) {
            startTimeSec = now;
            return 0;
        }

        double elapsed = now - startTimeSec;
        if (elapsed >= durationSeconds) {
            return DiagStatus32.TERM_TEST_TERMINATED_OK;
        }

        return 0;
    }
}
