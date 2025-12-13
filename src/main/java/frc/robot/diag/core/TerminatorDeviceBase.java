package frc.robot.diag.core;

/**
 * Base class for terminators that are ALSO full diag devices (limit switch, timer, etc.).
 *
 * Key rule:
 * - When acting as a terminator, it must NOT depend on its own Enable.
 *   Only the DUT's UseTerm_* gating matters.
 *
 * Generic behavior:
 * - DUT may call armForTest()/disarmForTest()
 * - getTerminatorStatus() returns 0 unless armed
 * - subclasses implement evalTerminatorStatus()
 * - subclasses may override getTerminatorDebug() to expose their raw/pressed/etc.
 *
 * Debug output:
 * - Terminator's own DebugState shows armed/hw/last + terminator-specific debug
 * - DUT DebugState shows UseTerm/armed/last per terminator (but not the raw details)
 */
public abstract class TerminatorDeviceBase extends DiagDeviceBase implements DiagTerminatorInterface {

    protected boolean termActive = false;
    private boolean termHardwareOpened = false;

    private int lastEvalStatus = 0;
    private String lastEvalDebug = "";

    protected TerminatorDeviceBase(String diagName) {
        super(diagName);
    }

    // Mandatory for all terminators
    @Override
    public final void armForTest() {
        termActive = true;

        // Ensure hardware exists even if this terminator's own Enable was never used.
        if (!termHardwareOpened) {
            int s = openHardware();
            termHardwareOpened = (DiagStatus32.getSeverity(s) < DiagStatus32.SEV_ERROR);
        }

        onTestStart();
    }

    // Mandatory for all terminators
    @Override
    public final void disarmForTest() {
        onTestEnd();
        termActive = false;

        // Close when the DUT test ends (deterministic; avoids stale WPILib objects)
        if (termHardwareOpened) {
            try {
                closeHardware();
            } catch (Exception ignored) {
            }
            termHardwareOpened = false;
        }
    }

    @Override
    public final int getTerminatorStatus() {
        if (!termActive) {
            return 0;
        }

        int s = evalTerminatorStatus();
        lastEvalStatus = s;

        String dbg = getTerminatorDebug();
        lastEvalDebug = (dbg != null) ? dbg : "";

        return s;
    }

    @Override
    public String getTerminatorName() {
        return getDiagName();
    }

    // Exposed for DUT-side debug summary (no raw details there)
    public final boolean isArmedForTest() {
        return termActive;
    }

    public final int getLastEvalStatus() {
        return lastEvalStatus;
    }

    public final String getLastEvalDebug() {
        return lastEvalDebug;
    }

    // Terminator devices show their own debug line even if their own Enable was never used.
    @Override
    protected final String getDebugStateExtra() {
        StringBuilder sb = new StringBuilder();
        sb.append("term_armed=").append(termActive ? "1" : "0");
        sb.append(" term_hw=").append(termHardwareOpened ? "1" : "0");
        sb.append(" term_last=").append(String.format("0x%08X", lastEvalStatus));

        if (lastEvalStatus != 0) {
            sb.append(" ").append(DiagStatus32.getMessage(lastEvalStatus));
        }

        if (lastEvalDebug != null && !lastEvalDebug.isEmpty()) {
            sb.append(" dbg=").append(lastEvalDebug);
        }

        return sb.toString();
    }

    // Subclasses implement termination logic ONLY.
    protected abstract int evalTerminatorStatus();

    // Must be public if DiagTerminatorInterface declares it.
    @Override
    public String getTerminatorDebug() {
        return "";
    }
}
