package frc.robot.diag.core;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for terminators that are ALSO full diag devices (limit switch, timer, etc.).
 *
 * Key rules:
 * - Terminator behavior does NOT depend on its own Enable. Only DUT UseTerm_* gating matters.
 * - Terminators return DiagStatus32.TERM_CONTINUE when not terminating.
 *
 * Shared terminators:
 * - Multiple DUTs may arm the same terminator instance at the same time.
 * - arm/disarm is reference-counted.
 * - When the terminator fires, it BROADCASTS to all bound DUTs immediately (Option 1).
 */
public abstract class TerminatorDeviceBase extends DiagDeviceBase
        implements DiagTerminatorInterface, DiagBindableTerminator {

    public enum TermRunState {
        UNOPENED,
        IDLE,
        ARMED_READY,
        TEST_WAIT,
        COMPLETE_GOOD,
        COMPLETE_BAD
    }

    private TermRunState termState = TermRunState.UNOPENED;

    // Shared-terminator support
    private int armRefCount = 0;

    // Who this terminator should terminate (Option 1)
    private final List<DiagDeviceBase> boundDuts = new ArrayList<>();
    private boolean broadcastedThisArm = false;

    // Debug / last eval info
    private int lastEvalStatus = DiagStatus32.TERM_CONTINUE;
    private String lastEvalDebug = "";

    // ----------------------------------------------------------------
    // Subclass hook: the actual termination condition
    // ----------------------------------------------------------------

    /**
     * Evaluate termination condition.
     *
     * Return:
     * - DiagStatus32.TERM_CONTINUE when not terminating
     * - DiagStatus32.TERM_TEST_TERMINATED_OK / TERM_TEST_TERMINATED_BAD when terminating
     * - Any SEV_ERROR status if the terminator itself hit a real error
     */
    protected abstract int evalTerminatorStatus();

    protected TerminatorDeviceBase(String diagName) {
        super(diagName);
    }

    // ----------------------------------------------------------------
    // Binding (Option 1)
    // ----------------------------------------------------------------

    @Override
    public final void bindDut(DiagDeviceBase dut) {
        if (dut == null) return;
        if (!boundDuts.contains(dut)) {
            boundDuts.add(dut);
        }
    }

    private void broadcastTerminateToBoundDuts(int termStatus) {

        if (broadcastedThisArm) return;
        broadcastedThisArm = true;

        String tn = getTerminatorName();

        for (DiagDeviceBase d : boundDuts) {
            try {
                d.requestTerminateFromTerminator(tn, termStatus);
            } catch (Exception ignored) {
            }
        }
    }

    // ----------------------------------------------------------------
    // DiagTerminatorInterface
    // ----------------------------------------------------------------

    @Override
    public final void armForTest() {

        armRefCount++;

        if (armRefCount > 1) {
            // Already armed by somebody else; do NOT reset state.
            return;
        }

        // New "arm session"
        broadcastedThisArm = false;
        lastEvalStatus = DiagStatus32.TERM_CONTINUE;
        lastEvalDebug = "";

        // Ensure hardware is opened once
        if (termState == TermRunState.UNOPENED) {
            int s = DiagStatus32.S_INIT_FAIL;
            try {
                s = openHardware();
            } catch (Exception ignored) {
            }

            if (DiagStatus32.getSeverity(s) >= DiagStatus32.SEV_ERROR) {
                termState = TermRunState.UNOPENED;
                lastEvalStatus = s;
                return;
            }

            termState = TermRunState.IDLE;
        }

        termState = TermRunState.ARMED_READY;
    }

    @Override
    public final void disarmForTest() {

        if (armRefCount <= 0) {
            armRefCount = 0;
            return;
        }

        armRefCount--;

        if (armRefCount > 0) {
            // Still armed by other DUT(s); do NOT reset.
            return;
        }

        // Nobody using us right now. Stay opened but idle.
        if (termState != TermRunState.UNOPENED) {
            termState = TermRunState.IDLE;
        }

        lastEvalStatus = DiagStatus32.TERM_CONTINUE;
        lastEvalDebug = "";
        broadcastedThisArm = false;
    }

    @Override
    public final int getTerminatorStatus() {

        // If not armed, do nothing.
        if (armRefCount <= 0) {
            return DiagStatus32.TERM_CONTINUE;
        }

        // If not opened, we can't evaluate.
        if (termState == TermRunState.UNOPENED) {
            return DiagStatus32.TERM_CONTINUE;
        }

        // First call after arm transitions into TEST_WAIT.
        if (termState == TermRunState.ARMED_READY) {
            termState = TermRunState.TEST_WAIT;
        }

        // Latched fired results
        if (termState == TermRunState.COMPLETE_GOOD || termState == TermRunState.COMPLETE_BAD) {
            return lastEvalStatus;
        }

        if (termState != TermRunState.TEST_WAIT) {
            return DiagStatus32.TERM_CONTINUE;
        }

        // Active evaluation
        int s = DiagStatus32.TERM_CONTINUE;
        try {
            s = evalTerminatorStatus();
        } catch (Exception ignored) {
            s = DiagStatus32.TERM_TEST_TERMINATED_BAD;
        }

        // Capture debug string (best-effort)
        try {
            String dbg = getTerminatorDebug();
            lastEvalDebug = (dbg != null) ? dbg : "";
        } catch (Exception ignored) {
            lastEvalDebug = "";
        }

        lastEvalStatus = s;

        // If the terminator itself errored, close and force UNOPENED.
        if (DiagStatus32.getSeverity(s) >= DiagStatus32.SEV_ERROR) {
            try {
                closeHardware();
            } catch (Exception ignored) {
            }
            termState = TermRunState.UNOPENED;
            return s;
        }

        // If it fired, latch and broadcast to all bound DUTs immediately (Option 1)
        if (s != DiagStatus32.TERM_CONTINUE) {

            if (s == DiagStatus32.TERM_TEST_TERMINATED_OK) {
                termState = TermRunState.COMPLETE_GOOD;
            } else {
                termState = TermRunState.COMPLETE_BAD;
            }

            broadcastTerminateToBoundDuts(s);
            return s;
        }

        return DiagStatus32.TERM_CONTINUE;
    }

    @Override
    public final String getTerminatorName() {
        return getDiagName();
    }

    @Override
    public String getTerminatorDebug() {
        return lastEvalDebug;
    }

    // ----------------------------------------------------------------
    // As a diag device (optional), we just keep ourselves "healthy"
    // ----------------------------------------------------------------

    @Override
    protected int runHardwareTest() {

        // Keep the terminator opened so it can be used quickly.
        if (termState == TermRunState.UNOPENED) {
            int s = DiagStatus32.S_INIT_FAIL;
            try {
                s = openHardware();
            } catch (Exception ignored) {
            }

            if (DiagStatus32.getSeverity(s) >= DiagStatus32.SEV_ERROR) {
                lastEvalStatus = s;
                return s;
            }

            termState = TermRunState.IDLE;
        }

        return DiagStatus32.S_TEST_OK;
    }

    @Override
    protected void stopHardware() {
        // no-op
    }

    @Override
    protected String getDebugStateExtra() {
        return "termState=" + termState
                + " arms=" + armRefCount
                + " bound=" + boundDuts.size()
                + " bcast=" + (broadcastedThisArm ? "1" : "0");
    }
}
