package frc.robot.diag.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DiagDeviceBase {

    // Global device list
    private static final List<DiagDeviceBase> DEVICES = new ArrayList<>();

    public static List<DiagDeviceBase> getDevices() {
        return Collections.unmodifiableList(DEVICES);
    }

    public static void periodicAll() {
        for (DiagDeviceBase d : DEVICES) {
            d.periodic();
        }
    }

    public static void stopAll() {
        for (DiagDeviceBase d : DEVICES) {
            d.stopTesting();
        }
    }

    // ----------------------------------------------------------------
    // Instance fields
    // ----------------------------------------------------------------

    protected final String diagName;
    protected final String baseKey;   // "Diag/<name>/"

    protected final DiagDeviceStatus status = new DiagDeviceStatus();

    private static final class TermBinding {
        final DiagTerminatorInterface term;
        final String useKey;        // Diag/<dut>/UseTerm_<termName>

        int lastTermStatus;         // last observed
        String lastTermDebug;       // last observed (if provided)

        // DUT-side arming latch: prevents repeated arm/disarm calls every periodic.
        boolean armed;

        TermBinding(DiagTerminatorInterface term, String useKey) {
            this.term = term;
            this.useKey = useKey;
            this.lastTermStatus = DiagStatus32.TERM_CONTINUE;
            this.lastTermDebug = "";
            this.armed = false;
        }
    }

    private final List<TermBinding> terminators = new ArrayList<>();

    // Status tracking
    private int lastInitStatus = DiagStatus32.S_UNSET;
    private int lastTestStatus = DiagStatus32.S_UNSET;
    private boolean everRan    = false;

    // Runtime state
    private boolean running       = false;
    private boolean testCompleted = false;

    // Who terminated the test (for StatusSummary)
    private String lastFiredTerminatorName = "";
    private int lastFiredTerminatorStatus = DiagStatus32.S_UNSET;

    // Dashboard init guard
    private boolean dashboardInitialized = false;

    // ----------------------------------------------------------------
    // Construction
    // ----------------------------------------------------------------

    public DiagDeviceBase(String diagName) {
        this.diagName = diagName;
        this.baseKey = "Diag/" + diagName + "/";
        DEVICES.add(this);
    }

    public String getDiagName() {
        return diagName;
    }

    // ----------------------------------------------------------------
    // Subclass API
    // ----------------------------------------------------------------

    protected abstract int openHardware();
    protected abstract void closeHardware();
    protected abstract int runHardwareTest();
    protected abstract void stopHardware();

    // Optional hooks (devices may override)
    protected void onTestStart() {}
    protected void onTestEnd() {}

    // Optional debug details hook (devices may override)
    protected String getDebugStateExtra() {
        return "";
    }

    // ----------------------------------------------------------------
    // Periodic
    // ----------------------------------------------------------------

    public void periodic() {

        // First time through, force dashboard keys to known defaults and
        // return immediately. This prevents a stale Enable=true from the
        // previous run from starting hardware the instant teleop enables.
        if (!dashboardInitialized) {
            updateDashboard(false, false);
            return;
        }

        boolean enable = SmartDashboard.getBoolean(baseKey + "Enable", false);
        boolean retry  = SmartDashboard.getBoolean(baseKey + "Retry", false);

        // Always publish, even if we return early
        updateDashboard(enable, retry);

        // Retry: edge-triggered behavior
        if (retry) {
            SmartDashboard.putBoolean(baseKey + "Retry", false);

            disarmAllTerminators();

            if (running) {
                stopHardware();
                onTestEnd();
                running = false;
            }
            closeHardware();

            lastInitStatus  = DiagStatus32.S_UNSET;
            lastTestStatus  = DiagStatus32.S_UNSET;
            everRan         = false;
            testCompleted   = false;

            lastFiredTerminatorName = "";
            lastFiredTerminatorStatus = DiagStatus32.S_UNSET;

            for (TermBinding b : terminators) {
                b.lastTermStatus = DiagStatus32.TERM_CONTINUE;
                b.lastTermDebug = "";
                b.armed = false;
            }

            updateDashboard(enable, false);
            return;
        }

        // Not enabled: stop device, clear completion latch so Enable can re-run
        if (!enable) {

            disarmAllTerminators();

            if (running) {
                stopHardware();
                onTestEnd();
                running = false;
            }
            testCompleted = false;

            // Clear "who terminated" when operator disables
            lastFiredTerminatorName = "";
            lastFiredTerminatorStatus = DiagStatus32.S_UNSET;

            updateDashboard(enable, retry);
            return;
        }

        // If a previous run ended due to terminator/fatal, do not auto-restart
        if (testCompleted) {
            updateDashboard(enable, retry);
            return;
        }

        // Start test if not running yet
        if (!running) {

            // Clear prior terminator attribution for the new run
            lastFiredTerminatorName = "";
            lastFiredTerminatorStatus = DiagStatus32.S_UNSET;

            lastInitStatus = openHardware();
            running = true;

            // We do NOT arm everything blindly here.
            // We arm based on UseTerm_* state transitions via syncTerminatorsArming().
            syncTerminatorsArming();

            onTestStart();

            if (DiagStatus32.getSeverity(lastInitStatus) >= DiagStatus32.SEV_ERROR) {
                lastTestStatus = lastInitStatus;
                everRan = true;

                disarmAllTerminators();

                stopHardware();
                onTestEnd();
                running = false;
                testCompleted = true;

                updateDashboard(enable, retry);
                return;
            }
        } else {
            // While running, allow UseTerm_* toggles to take effect immediately.
            // This only calls arm/disarm on transitions (no repeated work per cycle).
            syncTerminatorsArming();
        }

        // Check terminators first
        int termStatus = checkTerminators();
        if (termStatus != DiagStatus32.TERM_CONTINUE) {
            lastTestStatus = termStatus;
            everRan = true;

            disarmAllTerminators();

            stopHardware();
            onTestEnd();
            running = false;
            testCompleted = true;

            updateDashboard(enable, retry);
            return;
        }

        // Normal test step
        int stepStatus = runHardwareTest();
        lastTestStatus = stepStatus;
        everRan = true;

        updateDashboard(enable, retry);
    }

    public void stopTesting() {

        disarmAllTerminators();

        if (running) {
            stopHardware();
            onTestEnd();
            running = false;
        }
        testCompleted = false;

        lastFiredTerminatorName = "";
        lastFiredTerminatorStatus = DiagStatus32.S_UNSET;
    }

    // ----------------------------------------------------------------
    // Terminators
    // ----------------------------------------------------------------

    /**
     * Synchronize terminator arming with the current UseTerm_* toggle states.
     *
     * Key rule for performance/safety:
     * - armForTest()/disarmForTest() must only be called on transitions.
     *   This ensures terminators do not repeatedly open/close hardware or reset timers
     *   every periodic cycle.
     */
    private void syncTerminatorsArming() {

        for (TermBinding b : terminators) {

            boolean use = SmartDashboard.getBoolean(b.useKey, false);

            if (use && !b.armed) {
                // Rising edge: start using this terminator now
                try {
                    b.term.armForTest();
                } catch (Exception ignored) {
                }
                b.armed = true;
                // leave lastTermStatus/debug as-is until evaluated
            } else if (!use && b.armed) {
                // Falling edge: stop using this terminator now
                try {
                    b.term.disarmForTest();
                } catch (Exception ignored) {
                }
                b.armed = false;

                b.lastTermStatus = DiagStatus32.TERM_CONTINUE;
                b.lastTermDebug = "";
            }
        }
    }

    private void disarmAllTerminators() {
        for (TermBinding b : terminators) {
            if (b.armed) {
                try {
                    b.term.disarmForTest();
                } catch (Exception ignored) {
                }
                b.armed = false;
            }

            b.lastTermStatus = DiagStatus32.TERM_CONTINUE;
            b.lastTermDebug = "";
        }
    }

    private int checkTerminators() {

        for (TermBinding b : terminators) {

            boolean use = SmartDashboard.getBoolean(b.useKey, false);
            if (!use) {
                b.lastTermStatus = DiagStatus32.TERM_CONTINUE;
                b.lastTermDebug = "";
                continue;
            }

            // If UseTerm is true, syncTerminatorsArming() should have armed it already.
            // Still guard: if not armed for any reason, treat as continue.
            if (!b.armed) {
                b.lastTermStatus = DiagStatus32.TERM_CONTINUE;
                b.lastTermDebug = "";
                continue;
            }

            int s = b.term.getTerminatorStatus();
            b.lastTermStatus = s;

            String dbg = "";
            try {
                dbg = b.term.getTerminatorDebug();
            } catch (Exception ignored) {
            }
            b.lastTermDebug = (dbg != null) ? dbg : "";

            if (s != DiagStatus32.TERM_CONTINUE) {
                lastFiredTerminatorName = b.term.getTerminatorName();
                lastFiredTerminatorStatus = s;
                return s;
            }
        }

        return DiagStatus32.TERM_CONTINUE;
    }

    // B form: term + default UseTerm_* setting
    public void addTerminator(DiagTerminatorInterface term, boolean defaultUse) {
        if (term == null) {
            return;
        }

        String useKey = baseKey + "UseTerm_" + term.getTerminatorName();
        SmartDashboard.getEntry(useKey).setDefaultBoolean(defaultUse);

//        SmartDashboard.putBoolean(useKey, defaultUse);

        terminators.add(new TermBinding(term, useKey));
    }

    public void addTerminator(DiagTerminatorInterface term) {
        addTerminator(term, false);
    }

    // ----------------------------------------------------------------
    // Dashboard helpers
    // ----------------------------------------------------------------

    private void updateDashboard(boolean enable, boolean retry) {

        if (!dashboardInitialized) {
            dashboardInitialized = true;

            // Force known defaults on first contact

            SmartDashboard.getEntry(baseKey + "Enable").setDefaultBoolean(false);
            SmartDashboard.getEntry(baseKey + "Retry").setDefaultBoolean(false);
            
            SmartDashboard.getEntry(baseKey + "LastStatusHex").setDefaultString(String.format("0x%08X", DiagStatus32.S_UNSET));
            SmartDashboard.getEntry(baseKey + "Health").setDefaultString("UNKNOWN");
            SmartDashboard.getEntry(baseKey + "StatusSummary").setDefaultString("test not run");
            SmartDashboard.getEntry(baseKey + "State").setDefaultString("IDLE");
            SmartDashboard.getEntry(baseKey + "DebugState").setDefaultString("init");
            
            // SmartDashboard.putBoolean(baseKey + "Enable", false);
            // SmartDashboard.putBoolean(baseKey + "Retry",  false);
            
            SmartDashboard.putString(baseKey + "LastStatusHex", String.format("0x%08X", DiagStatus32.S_UNSET));
            SmartDashboard.putString(baseKey + "Health",        "UNKNOWN");
            SmartDashboard.putString(baseKey + "StatusSummary", "test not run");
            SmartDashboard.putString(baseKey + "State",         "IDLE");
            SmartDashboard.putString(baseKey + "DebugState",    "init");
        }

        status.setLastStatus(lastTestStatus);

        SmartDashboard.putString(baseKey + "LastStatusHex", status.getLastStatusHex());
        SmartDashboard.putString(baseKey + "Health",        computeHealth());
        SmartDashboard.putString(baseKey + "StatusSummary", computeSummary());

        SmartDashboard.putString(baseKey + "State",      computeState(enable, retry));
        SmartDashboard.putString(baseKey + "DebugState", computeDebugState(enable, retry));
    }

    private String computeState(boolean enable, boolean retry) {
        if (!enable) {
            return "IDLE";
        }
        if (retry) {
            return "RETRY";
        }
        if (running) {
            return "TESTING";
        }
        if (testCompleted) {
            int sev = DiagStatus32.getSeverity(lastTestStatus);
            if (sev == DiagStatus32.SEV_SUCCESS || sev == DiagStatus32.SEV_INFO) {
                return "FINISHED_GOOD";
            }
            if (sev >= DiagStatus32.SEV_ERROR) {
                return "FINISHED_BAD";
            }
            return "FINISHED_WARN";
        }
        return "INIT";
    }

    private String computeDebugState(boolean enable, boolean retry) {

        StringBuilder sb = new StringBuilder();

        sb.append("enable=").append(enable ? "1" : "0");
        sb.append(" retry=").append(retry ? "1" : "0");
        sb.append(" running=").append(running ? "1" : "0");
        sb.append(" completed=").append(testCompleted ? "1" : "0");
        sb.append(" everRan=").append(everRan ? "1" : "0");

        sb.append(" init=").append(String.format("0x%08X", lastInitStatus));
        sb.append(" test=").append(String.format("0x%08X", lastTestStatus));

        if (!lastFiredTerminatorName.isEmpty()) {
            sb.append(" fired=").append(lastFiredTerminatorName);
        }

        if (!terminators.isEmpty()) {
            sb.append(" terms=").append(terminators.size());
            for (TermBinding b : terminators) {
                boolean use = SmartDashboard.getBoolean(b.useKey, false);
                sb.append(" [");
                sb.append(b.term.getTerminatorName());
                sb.append(" use=").append(use ? "1" : "0");
                sb.append(" armed=").append(b.armed ? "1" : "0");
                sb.append(" last=").append(String.format("0x%08X", b.lastTermStatus));
                if (b.lastTermDebug != null && !b.lastTermDebug.isEmpty()) {
                    sb.append(" dbg=").append(b.lastTermDebug);
                }
                sb.append("]");
            }
        }

        String extra = getDebugStateExtra();
        if (extra != null && !extra.isEmpty()) {
            sb.append(" | ").append(extra);
        }

        return sb.toString();
    }

    private String computeHealth() {
        if (!everRan) {
            return "UNKNOWN";
        }

        int sev = DiagStatus32.getSeverity(lastTestStatus);

        if (sev == DiagStatus32.SEV_SUCCESS || sev == DiagStatus32.SEV_INFO) {
            return "GOOD";
        } else if (sev >= DiagStatus32.SEV_ERROR) {
            return "BAD";
        } else {
            return "POSSIBLY_BAD";
        }
    }

    private String computeSummary() {
        if (!everRan) {
            return "test not run";
        }

        int sev = DiagStatus32.getSeverity(lastTestStatus);
        String prefix;
        if (sev == DiagStatus32.SEV_SUCCESS || sev == DiagStatus32.SEV_INFO) {
            prefix = "G: ";
        } else if (sev >= DiagStatus32.SEV_ERROR) {
            prefix = "E: ";
        } else {
            prefix = "W: ";
        }

        String msg = DiagStatus32.getMessage(lastTestStatus);

        // Add "who terminated" when termination was caused by a terminator
        if (!lastFiredTerminatorName.isEmpty() && lastTestStatus == lastFiredTerminatorStatus) {
            msg = msg + " (by " + lastFiredTerminatorName + ")";
        }

        return prefix + msg;
    }
}
