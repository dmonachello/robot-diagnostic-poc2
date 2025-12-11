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
    // Terminator binding
    // ----------------------------------------------------------------

    private static class TerminatorBinding {
        final DiagTerminatorInterface terminator;
        final String useKey;  // SmartDashboard key for "UseAsTerminator"

        TerminatorBinding(DiagTerminatorInterface terminator, String useKey) {
            this.terminator = terminator;
            this.useKey = useKey;
        }
    }

    // ----------------------------------------------------------------
    // Instance fields
    // ----------------------------------------------------------------

    protected final String diagName;
    protected final String baseKey;   // "Diag/<name>/"

    protected final DiagDeviceStatus status = new DiagDeviceStatus();

    private final List<TerminatorBinding> terminators = new ArrayList<>();

    // Status tracking
    private int  lastInitStatus = 0;
    private int  lastTestStatus = 0;
    private boolean everRan     = false;

    // Which terminator last stopped this test (if any)
    private String lastTerminatorName = null;

    // Runtime state
    private boolean running       = false;
    private boolean testCompleted = false;

    // Dashboard init guard
    private boolean dashboardInitialized = false;

    // Track previous Enable to detect rising edge
    private boolean lastEnable = false;

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

    protected abstract int  openHardware();
    protected abstract void closeHardware();
    protected abstract int  runHardwareTest();
    protected abstract void stopHardware();

    protected void onTestStart() {}
    protected void onTestEnd()   {}

    // ----------------------------------------------------------------
    // Periodic
    // ----------------------------------------------------------------

    public void periodic() {
        updateDashboard();

        boolean enable = SmartDashboard.getBoolean(baseKey + "Enable", false);
        boolean retry  = SmartDashboard.getBoolean(baseKey + "Retry",  false);
        boolean risingEnable = enable && !lastEnable;

        // Retry: full reset and re-open on next enabled cycle
        if (retry) {
            SmartDashboard.putBoolean(baseKey + "Retry", false);

            if (running) {
                stopHardware();
                onTestEnd();
                running = false;
            }
            closeHardware();

            lastInitStatus      = 0;
            lastTestStatus      = 0;
            everRan             = false;
            testCompleted       = false;
            lastTerminatorName  = null;

            updateDashboard();
            lastEnable = enable;
            return;
        }

        // If Enable just transitioned from false -> true,
        // treat that as "start a fresh test".
        if (risingEnable) {
            lastInitStatus      = 0;
            lastTestStatus      = 0;
            everRan             = false;
            testCompleted       = false;
            lastTerminatorName  = null;
        }

        // Not enabled: make sure we are not running and just publish current state
        if (!enable) {
            if (running) {
                stopHardware();
                onTestEnd();
                running = false;
            }
            updateDashboard();
            lastEnable = enable;
            return;
        }

        // Enabled but test was already completed
        if (testCompleted) {
            updateDashboard();
            lastEnable = enable;
            return;
        }

        // Start test if not running yet
        if (!running) {
            lastInitStatus = openHardware();
            running = true;
            onTestStart();

            if (DiagStatus32.getSeverity(lastInitStatus) >= DiagStatus32.SEV_ERROR) {
                lastTestStatus = lastInitStatus;
                everRan = true;

                stopHardware();
                onTestEnd();
                running = false;
                testCompleted = true;

                updateDashboard();
                lastEnable = enable;
                return;
            }
        }

        // Check terminators first
        int termStatus = checkTerminators();
        if (termStatus != 0) {
            lastTestStatus = termStatus;
            everRan = true;

            stopHardware();
            onTestEnd();
            running = false;
            testCompleted = true;

            updateDashboard();
            lastEnable = enable;
            return;
        }

        // Normal test step
        int stepStatus = runHardwareTest();
        lastTestStatus = stepStatus;
        everRan = true;

        updateDashboard();
        lastEnable = enable;
    }

    public void stopTesting() {
        if (running) {
            stopHardware();
            onTestEnd();
            running = false;
        }
    }

    // ----------------------------------------------------------------
    // Terminators
    // ----------------------------------------------------------------

    private int checkTerminators() {
        for (TerminatorBinding b : terminators) {
            boolean use = SmartDashboard.getBoolean(b.useKey, true);
            if (!use) {
                continue;
            }

            int s = b.terminator.getTerminatorStatus();
            if (s != 0) {
                lastTerminatorName = b.terminator.getTerminatorName();
                return s;
            }
        }
        return 0;
    }

    public void addTerminator(DiagTerminatorInterface term) {
        addTerminator(term, true);
    }

    public void addTerminator(DiagTerminatorInterface term, boolean defaultUse) {
        if (term != null) {
            String safeName = term.getTerminatorName().replace(' ', '_');
            String key = baseKey + "UseTerm_" + safeName;

            TerminatorBinding binding = new TerminatorBinding(term, key);
            terminators.add(binding);

            SmartDashboard.setDefaultBoolean(key, defaultUse);
        }
    }

    // ----------------------------------------------------------------
    // Dashboard helpers
    // ----------------------------------------------------------------

    private void updateDashboard() {
        if (!dashboardInitialized) {
            dashboardInitialized = true;
            SmartDashboard.putBoolean(baseKey + "Enable", false);
            SmartDashboard.putBoolean(baseKey + "Retry",  false);
        }

        status.setLastStatus(lastTestStatus);

        SmartDashboard.putString(baseKey + "LastStatusHex", status.getLastStatusHex());
        SmartDashboard.putString(baseKey + "Health",        computeHealth());
        SmartDashboard.putString(baseKey + "StatusSummary", computeSummary());
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

        String base = DiagStatus32.getMessage(lastTestStatus);

        // If we know which terminator fired, always append it.
        if (lastTerminatorName != null) {
            base = base + " (terminator = " + lastTerminatorName + ")";
        }

        return prefix + base;
    }
}
