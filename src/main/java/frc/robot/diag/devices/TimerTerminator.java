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
 *    TERM_TEST_TERMINATED_OK.
 */
public class TimerTerminator extends TerminatorDeviceBase {

    private final double durationSeconds;

    // Per-test timing
    private double startTimeSec = -1.0;

    // Debug sampling
    private double lastElapsedSec = 0.0;

    public TimerTerminator(String diagName, double durationSeconds) {
        super(diagName);
        this.durationSeconds = durationSeconds;
    }

    // ---------------- Diag device side ----------------

    @Override
    protected int openHardware() {
        startTimeSec = -1.0;
        lastElapsedSec = 0.0;
        return DiagStatus32.S_INIT_OK;
    }

    @Override
    protected void closeHardware() {
        // Nothing to close
    }

    @Override
    protected int runHardwareTest() {
        return DiagStatus32.S_TEST_OK;
    }

    @Override
    protected void stopHardware() {
        // Nothing to stop
    }

    // ---------------- Terminator lifecycle ----------------

    @Override
    public void onTestStart() {
        startTimeSec = -1.0;
        lastElapsedSec = 0.0;
    }

    @Override
    public void onTestEnd() {
        // leave values; reset on next test
    }

    // ---------------- Terminator evaluation ----------------

    @Override
    protected int evalTerminatorStatus() {
        double now = Timer.getFPGATimestamp();

        if (startTimeSec < 0.0) {
            startTimeSec = now;
            lastElapsedSec = 0.0;
            return 0;
        }

        lastElapsedSec = now - startTimeSec;

        if (lastElapsedSec >= durationSeconds) {
            return DiagStatus32.TERM_TEST_TERMINATED_OK;
        }

        return 0;
    }

    @Override
    public String getTerminatorDebug() {
        return String.format("elapsed=%.2f dur=%.2f",
                lastElapsedSec, durationSeconds);
    }
}
