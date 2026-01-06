package frc.robot.diag.devices;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.diag.core.DiagStatus32;
import frc.robot.diag.core.TerminatorDeviceBase;

/**
 * Time-based terminator that can be shared across multiple DUTs.
 *
 * Behavior:
 * - Epoch is the global StartTest time (Diag/RunStartTimeSec).
 * - Epoch resets automatically when Diag/RunId changes.
 * - Works correctly when shared: multiple DUTs can arm the same instance.
 */
public class TimerTerminator extends TerminatorDeviceBase {

    private static final String GLOBAL_RUNID_KEY    = "Diag/RunId";
    private static final String GLOBAL_RUNSTART_KEY = "Diag/RunStartTimeSec";

    private final double durationSeconds;

    private int lastRunIdSeen = -1;

    private double epochSec = -1.0;
    private double lastElapsedSec = 0.0;

    public TimerTerminator(String diagName, double durationSeconds) {
        super(diagName);
        this.durationSeconds = durationSeconds;
    }

    @Override
    protected int openHardware() {
        lastRunIdSeen = -1;
        epochSec = -1.0;
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

    private void refreshEpochIfNewRun(double now) {

        int runId = (int) SmartDashboard.getNumber(GLOBAL_RUNID_KEY, -1);

        if (runId != lastRunIdSeen) {
            lastRunIdSeen = runId;

            double globalEpoch = SmartDashboard.getNumber(GLOBAL_RUNSTART_KEY, -1.0);
            if (globalEpoch > 0.0) {
                epochSec = globalEpoch;
            } else {
                // Fallback if RunStartTime isn't present yet
                epochSec = now;
            }

            lastElapsedSec = 0.0;
        }

        // Safety: if epoch is still invalid, set it
        if (epochSec < 0.0) {
            epochSec = now;
            lastElapsedSec = 0.0;
        }
    }

    @Override
    protected int evalTerminatorStatus() {

        double now = Timer.getFPGATimestamp();

        // Critical fix: reset epoch when a new run starts
        refreshEpochIfNewRun(now);

        lastElapsedSec = now - epochSec;

        if (lastElapsedSec >= durationSeconds) {
            return DiagStatus32.TERM_TEST_TERMINATED_OK;
        }

        return DiagStatus32.TERM_CONTINUE;
    }

    @Override
    public String getTerminatorDebug() {
        return "elapsed=" + String.format("%.3f", lastElapsedSec)
             + " dur=" + String.format("%.3f", durationSeconds)
             + " runId=" + lastRunIdSeen;
    }
}
