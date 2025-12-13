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

    // Called by the DUT when this terminator is being used for a test run.
    void armForTest();

    // Called by the DUT when this terminator is no longer being used for a test run.
    void disarmForTest();

    /**
     * Returns a status code:
     * - DiagStatus32.TERM_CONTINUE means "keep going"
     * - any other value means "terminate now using that status"
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
