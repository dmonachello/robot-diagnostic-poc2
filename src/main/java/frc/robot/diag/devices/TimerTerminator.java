package frc.robot.diag.devices;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.diag.core.DiagStatus32;
import frc.robot.diag.core.TerminatorDeviceBase;

/**
 * Pure time-based terminator that can also show up as a diag device.
 */
public class TimerTerminator extends TerminatorDeviceBase {

    private final double durationSeconds;
    private double startTimeSec = -1.0;
    private double lastElapsedSec = 0.0;

    public TimerTerminator(String diagName, double durationSeconds) {
        super(diagName);
        this.durationSeconds = durationSeconds;
    }

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

    @Override
    public void onTestStart() {
        startTimeSec = -1.0;
        lastElapsedSec = 0.0;
    }

    @Override
    protected int evalTerminatorStatus() {

        double now = Timer.getFPGATimestamp();

        if (startTimeSec < 0.0) {
            startTimeSec = now;
            lastElapsedSec = 0.0;
            return DiagStatus32.TERM_CONTINUE;
        }

        lastElapsedSec = now - startTimeSec;

        if (lastElapsedSec >= durationSeconds) {
            return DiagStatus32.TERM_TEST_TERMINATED_OK;
        }

        return DiagStatus32.TERM_CONTINUE;
    }

    @Override
    public String getTerminatorDebug() {
        return "elapsed=" + String.format("%.3f", lastElapsedSec)
             + " dur=" + String.format("%.3f", durationSeconds);
    }
}
