package frc.robot.diag.core;

/**
 * A test terminator: something that can say "stop this test now".
 *
 * Key rule:
 * - When acting as a terminator, it must NOT depend on its own Enable.
 *   Only the DUT's UseTerm_* gating matters.
 *
 * Sentinel rule:
 * - Terminators must not return 0.
 * - When the terminator does NOT want to stop the DUT, it returns TERM_CONTINUE.
 */
public interface DiagTerminatorInterface {

    /**
     * Called by the DUT when this terminator is entering active use for a test run.
     * Terminators should (re)open hardware here if needed and reset any per-run state.
     */
    void armForTest();

    /**
     * Called by the DUT when this terminator is no longer being used for the current run.
     * Terminators may stop polling hardware, etc.
     */
    void disarmForTest();

    /**
     * Return a terminator status code.
     * Use DiagStatus32.TERM_CONTINUE when the terminator does NOT want to stop the DUT.
     */
    int getTerminatorStatus();

    /**
     * Name to show in UseTerm_* keys and logs.
     */
    String getTerminatorName();

    /**
     * Optional debug string for the DUT debug line.
     */
    default String getTerminatorDebug() {
        return "";
    }
}
