package frc.robot.diag.core;

/**
 * Base class for devices that are ALSO test terminators.
 *
 * Examples:
 *  - Limit switches
 *  - Beam-break sensors
 *  - Timers
 *
 * This class:
 *  - Handles the "armed/unarmed" lifecycle for use as a terminator.
 *  - Provides a final getTerminatorStatus() that gates on that lifecycle.
 *  - Lets subclasses implement one method: evalTerminatorStatus().
 */
public abstract class TerminatorDeviceBase extends DiagDeviceBase
                                          implements DiagTerminatorInterface {

    // True while this object is acting as a terminator for some DUT's test.
    protected boolean termActive = false;

    protected TerminatorDeviceBase(String diagName) {
        super(diagName);
    }

    // Called when a DUT test starts (by DiagDeviceBase on the owning device)
    @Override
    public void onTestStart() {
        termActive = true;
    }

    // Called when a DUT test ends (disable, retry, fault, etc.)
    @Override
    public void onTestEnd() {
        termActive = false;
    }

    /**
     * Final implementation used by DiagDeviceBase when this object is
     * attached as a terminator to a DUT.
     *
     * We only delegate to evalTerminatorStatus() when termActive is true.
     */
    @Override
    public final int getTerminatorStatus() {
        if (!termActive) {
            return 0;
        }
        return evalTerminatorStatus();
    }

    /**
     * Subclasses implement this to provide their actual terminator logic.
     *
     * Rules:
     *  - Return 0 if you do NOT want to stop the owning test.
     *  - Return TERM_TEST_TERMINATED_OK or TERM_TEST_TERMINATED_BAD
     *    (or a facility-specific variant) when you want the test to stop.
     */
    protected abstract int evalTerminatorStatus();
}
