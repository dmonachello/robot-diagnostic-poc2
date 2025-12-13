package frc.robot.diag.core;

/**
 * Base class for terminators that are ALSO full diag devices (limit switch, timer, etc.).
 *
 * Key rules:
 * - When acting as a terminator, it must NOT depend on its own Enable.
 *   Only the DUT's UseTerm_* gating matters.
 * - Terminators must not return 0. They return DiagStatus32.TERM_CONTINUE to mean "keep going".
 *
 * State model (single state variable):
 * - UNOPENED: hardware not opened
 * - IDLE:     hardware opened, not actively being used by a DUT test
 * - ARMED_READY: armed for a DUT run (about to start waiting)
 * - TEST_WAIT: actively evaluating termination condition
 * - COMPLETE_GOOD / COMPLETE_BAD: fired and latched for the current run
 *
 * Error behavior (what you asked for):
 * - If evalTerminatorStatus() returns a status with severity >= SEV_ERROR:
 *     - closeHardware() immediately (once)
 *     - transition to UNOPENED
 *     - next armForTest() will reopen
 */
public abstract class TerminatorDeviceBase extends DiagDeviceBase implements DiagTerminatorInterface {

    public enum TermRunState {
        UNOPENED,
        IDLE,
        ARMED_READY,
        TEST_WAIT,
        COMPLETE_GOOD,
        COMPLETE_BAD
    }

    private TermRunState termState = TermRunState.UNOPENED;

    // Debug + counters
    private int lastEvalStatus = DiagStatus32.TERM_CONTINUE;
    private String lastEvalDebug = "";

    private int openOkCount = 0;
    private int openFailCount = 0;
    private int closeOkCount = 0;
    private int closeFailCount = 0;

    private int firedGoodCount = 0;
    private int firedBadCount = 0;

    protected TerminatorDeviceBase(String diagName) {
        super(diagName);
    }

    // ----------------------------------------------------------------
    // DiagTerminatorInterface
    // ----------------------------------------------------------------

    @Override
    public final void armForTest() {

        // Ensure we are opened (but do not flap open/close every cycle).
        if (termState == TermRunState.UNOPENED) {
            int s = DiagStatus32.S_INIT_FAIL;
            try {
                s = openHardware();
            } catch (Exception ignored) {
            }

            if (DiagStatus32.getSeverity(s) >= DiagStatus32.SEV_ERROR) {
                openFailCount++;
                // Stay UNOPENED. DUT can still "use" us but we'll never fire.
                termState = TermRunState.UNOPENED;
                lastEvalStatus = s;
                lastEvalDebug = "";
                return;
            }

            openOkCount++;
            termState = TermRunState.IDLE;
        }

        // New run: clear latch
        lastEvalStatus = DiagStatus32.TERM_CONTINUE;
        lastEvalDebug = "";

        termState = TermRunState.ARMED_READY;
        onTestStart();
    }

    @Override
    public final void disarmForTest() {

        onTestEnd();

        // If we fired (good/bad), go back to IDLE and stay opened.
        // If we were forced UNOPENED due to error, leave it UNOPENED.
        if (termState == TermRunState.COMPLETE_GOOD || termState == TermRunState.COMPLETE_BAD) {
            termState = TermRunState.IDLE;
        } else if (termState == TermRunState.ARMED_READY || termState == TermRunState.TEST_WAIT) {
            termState = TermRunState.IDLE;
        }
    }

    @Override
    public final int getTerminatorStatus() {

        // Not armed/active for a run
        if (termState == TermRunState.UNOPENED || termState == TermRunState.IDLE) {
            lastEvalStatus = DiagStatus32.TERM_CONTINUE;
            lastEvalDebug = "";
            return DiagStatus32.TERM_CONTINUE;
        }

        // First evaluation tick after arming
        if (termState == TermRunState.ARMED_READY) {
            termState = TermRunState.TEST_WAIT;
        }

        // If already fired this run, keep returning the fired status.
        if (termState == TermRunState.COMPLETE_GOOD || termState == TermRunState.COMPLETE_BAD) {
            return lastEvalStatus;
        }

        // Normal evaluation
        int s;
        try {
            s = evalTerminatorStatus();
        } catch (Exception e) {
            s = DiagStatus32.TERM_TEST_TERMINATED_BAD;
        }

        lastEvalStatus = s;

        String dbg = "";
        try {
            dbg = getTerminatorDebug();
        } catch (Exception ignored) {
        }
        lastEvalDebug = (dbg != null) ? dbg : "";

        // Keep going sentinel
        if (s == DiagStatus32.TERM_CONTINUE) {
            return DiagStatus32.TERM_CONTINUE;
        }

        // Fired: determine good/bad by severity
        int sev = DiagStatus32.getSeverity(s);

        if (sev >= DiagStatus32.SEV_ERROR) {
            firedBadCount++;

            // Requested change: close immediately ONCE, and force UNOPENED so next arm reopens.
            // Guard: only close if we are currently in an opened/testing state.
            boolean didClose = false;
            if (termState != TermRunState.UNOPENED) {
                try {
                    closeHardware();
                    closeOkCount++;
                    didClose = true;
                } catch (Exception ignored) {
                    closeFailCount++;
                    didClose = true; // we attempted
                }
            }

            termState = TermRunState.UNOPENED;

            // If we didn't even attempt a close (shouldn't happen), at least keep state consistent.
            if (!didClose) {
                // nothing else
            }

            // Latch as "complete bad" logically for this run, but hardware state is UNOPENED.
            // The DUT will disarm us; next run will reopen.
            // (We keep returning the error status.)
            return s;
        }

        // Good termination (success/info/warn but non-continue)
        firedGoodCount++;
        termState = TermRunState.COMPLETE_GOOD;
        return s;
    }

    @Override
    public final String getTerminatorName() {
        return getDiagName();
    }

    // ----------------------------------------------------------------
    // Debug
    // ----------------------------------------------------------------

    @Override
    protected final String getDebugStateExtra() {
        StringBuilder sb = new StringBuilder();
        sb.append("term_state=").append(termState);
        sb.append(" open_ok=").append(openOkCount);
        sb.append(" open_fail=").append(openFailCount);
        sb.append(" close_ok=").append(closeOkCount);
        sb.append(" close_fail=").append(closeFailCount);
        sb.append(" fired_good=").append(firedGoodCount);
        sb.append(" fired_bad=").append(firedBadCount);
        sb.append(" last=").append(String.format("0x%08X", lastEvalStatus));
        sb.append(" ").append(DiagStatus32.getMessage(lastEvalStatus));
        if (lastEvalDebug != null && !lastEvalDebug.isEmpty()) {
            sb.append(" dbg=").append(lastEvalDebug);
        }
        return sb.toString();
    }

    // ----------------------------------------------------------------
    // Subclass API
    // ----------------------------------------------------------------

    // Subclasses implement termination logic ONLY:
    // - return DiagStatus32.TERM_CONTINUE to keep going
    // - return any other status to terminate (good/bad determined by severity)
    protected abstract int evalTerminatorStatus();

    @Override
    public String getTerminatorDebug() {
        return "";
    }
}
