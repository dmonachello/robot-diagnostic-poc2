package frc.robot.diag.core;

/**
 * A test terminator: something that can say "stop this test now".
 *
 * Key rule:
 * - When acting as a terminator, it must NOT depend on its own Enable.
 *   Only the DUT's UseTerm_* gating matters.
 */
public interface DiagTerminatorInterface {

    // Called by the DUT when this terminator is being used for a test run.
    void armForTest();

    // Called by the DUT when this terminator is no longer being used for a test run.
    void disarmForTest();

    /**
     * Returns a status code when this terminator wants the owning test to stop.
     * 0 means "no termination".
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
