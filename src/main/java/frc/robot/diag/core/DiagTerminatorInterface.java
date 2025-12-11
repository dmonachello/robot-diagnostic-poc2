package frc.robot.diag.core;

/**
 * A test terminator: something that can say "stop this test now".
 *
 * Devices like limit switches, beam breaks, timers, etc. implement this.
 * The same object can also be a full diagnostic device (extends DiagDeviceBase).
 */
public interface DiagTerminatorInterface {

    /**
     * Returns a status code when this terminator wants the owning test to stop.
     * 0 means "no termination".
     */
    int getTerminatorStatus();

    /**
     * Name to show in logs / status when this terminator fires.
     */
    String getTerminatorName();

    /**
     * Called when a DUT test that uses this terminator starts.
     * (DiagDeviceBase calls this only if the DUT's UseTerm_* flag is true.)
     */
    void onTestStart();

    /**
     * Called when that DUT test ends (disabled, retry, or terminated).
     */
    void onTestEnd();
}
