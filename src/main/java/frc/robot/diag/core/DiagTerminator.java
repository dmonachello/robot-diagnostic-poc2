package frc.robot.diag.core;

/**
 * A test terminator: something that can say "stop this test now".
 *
 * Devices like limit switches, beam breaks, timers, etc. can implement this.
 * The same object can also be a full diagnostic device (extends DiagDeviceBase).
 */
public interface DiagTerminator {

    /**
     * Returns a status code when this terminator wants the owning test to stop.
     * 0 means "no termination".
     *
     * Typical values:
     *   TERM_TEST_TERMINATED_OK  - hit safety limit, everything fine
     *   TERM_TEST_TERMINATED_BAD - hit error condition
     */
    int getTerminatorStatus();

    /**
     * Name to show in logs / status when this terminator fires.
     */
    String getTerminatorName();
}
