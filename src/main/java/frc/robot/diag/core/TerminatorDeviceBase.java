package frc.robot.diag.core;

/**
 * Base class for terminators that are ALSO full diag devices
 * (limit switch, timer, etc.).
 *
 * It provides default no-op implementations of the lifecycle hooks so
 * subclasses only implement what they need.
 */
public abstract class TerminatorDeviceBase extends DiagDeviceBase
                                          implements DiagTerminatorInterface {

    protected TerminatorDeviceBase(String diagName) {
        super(diagName);
    }

    // Subclasses must implement the actual termination logic
    @Override
    public abstract int getTerminatorStatus();

    @Override
    public String getTerminatorName() {
        return getDiagName();
    }

    @Override
    public void onTestStart() {
        // default: nothing; subclasses override if they need a per-test reset
    }

    @Override
    public void onTestEnd() {
        // default: nothing; subclasses override if they need cleanup
    }
}
