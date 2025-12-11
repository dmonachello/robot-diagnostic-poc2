package frc.robot.diag.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DiagDeviceBase {

    // --------------------------------------------------------------
    // Global device list
    // --------------------------------------------------------------

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

    // --------------------------------------------------------------
    // Instance fields
    // --------------------------------------------------------------

    protected final String diagName;
    protected final String baseKey;   // "Diag/<name>/"

    protected final DiagDeviceStatus status = new DiagDeviceStatus();

    private static class TerminatorBinding {
        final DiagTerminatorInterface term;
        final String label;
        final boolean defaultUse;

        TerminatorBinding(DiagTerminatorInterface term, String label, boolean defaultUse) {
            this.term = term;
            this.label = label;
            this.defaultUse = defaultUse;
        }
    }

    private final List<TerminatorBinding> terminators = new ArrayList<>();

    // Status tracking
    private int  lastInitStatus = 0;
    private int  lastTestStatus = 0;
    private boolean everRan     = false;

    // Runtime state
    private boolean running       = false;
    private boolean testCompleted = false;

    // Dashboard init guard
    private boolean dashboardInitialized = false;

    // Which terminator stopped the last test (if any)
    private String lastTerminatorLabel = null;

    // --------------------------------------------------------------
    // Construction
    // --------------------------------------------------------------

    public DiagDeviceBase(String diagName) {
        this.diagName = diagName;
        this.baseKey  = "Diag/" + diagName + "/";
        DEVICES.add(this);
    }

    public String getDiagName() {
        return diagName;
    }

    // --------------------------------------------------------------
    // Subclass API
    // --------------------------------------------------------------

    protected abstract int  openHardware();
    protected abstract void closeHardware();
    protected abstract int  runHardwareTest();
    protected abstract void stopHardware();

    // Optional hooks for DUTs (not terminators)
    public void onTestStart() {}
    public void onTestEnd()   {}

    // --------------------------------------------------------------
    // Main periodic
    // --------------------------------------------------------------

    public void periodic() {
        ensureDashboardKeys();
        updateDashboard();

        boolean enable = SmartDashboard.getBoolean(baseKey + "Enable", false);
        boolean retry  = SmartDashboard.getBoolean(baseKey + "Retry",  false);

        // Retry: full reset and re-open on next enabled cycle
        if (retry) {
            SmartDashboard.putBoolean(baseKey + "Retry", false);

            if (running) {
                stopHardware();
                onTestEnd();
                notifyTerminatorsEnd();
                running = false;
            }
            closeHardware();

            lastInitStatus      = 0;
            lastTestStatus      = 0;
            everRan             = false;
            testCompleted       = false;
            lastTerminatorLabel = null;

            updateDashboard();
            return;
        }

        // Not enabled: stop if running, but keep last result
        if (!enable) {
            if (running) {
                stopHardware();
                onTestEnd();
                notifyTerminatorsEnd();
                running = false;
            }
            updateDashboard();
            return;
        }

        // Already completed (by terminator or fatal init)
        if (testCompleted) {
            updateDashboard();
            return;
        }

        // First time after enable → init
        if (!running) {
            lastInitStatus = openHardware();
            running = true;

            onTestStart();
            notifyTerminatorsStart();

            // Init failure counts as a completed test
            if (DiagStatus32.getSeverity(lastInitStatus) >= DiagStatus32.SEV_ERROR) {
                lastTestStatus = lastInitStatus;
                everRan        = true;

                stopHardware();
                onTestEnd();
                notifyTerminatorsEnd();
                running       = false;
                testCompleted = true;

                updateDashboard();
                return;
            }
        }

        // Check terminators first
        lastTerminatorLabel = null;
        int termStatus = checkTerminators();
        if (termStatus != 0) {
            lastTestStatus = termStatus;
            everRan        = true;

            stopHardware();
            onTestEnd();
            notifyTerminatorsEnd();
            running       = false;
            testCompleted = true;

            updateDashboard();
            return;
        }

        // Normal test step
        int stepStatus = runHardwareTest();
        lastTestStatus = stepStatus;
        everRan        = true;

        updateDashboard();
    }

    public void stopTesting() {
        if (running) {
            stopHardware();
            onTestEnd();
            notifyTerminatorsEnd();
            running = false;
        }
    }

    // --------------------------------------------------------------
    // Terminators
    // --------------------------------------------------------------

    private int checkTerminators() {
        for (TerminatorBinding b : terminators) {
            String key = baseKey + "UseTerm_" + b.label;
            boolean use = SmartDashboard.getBoolean(key, b.defaultUse);
            SmartDashboard.putBoolean(key, use);   // keep NT in sync

            if (!use) {
                continue;
            }

            int s = b.term.getTerminatorStatus();
            if (s != 0) {
                lastTerminatorLabel = b.label;
                return s;
            }
        }
        return 0;
    }

    private void notifyTerminatorsStart() {
        for (TerminatorBinding b : terminators) {
            String key = baseKey + "UseTerm_" + b.label;
            boolean use = SmartDashboard.getBoolean(key, b.defaultUse);
            SmartDashboard.putBoolean(key, use);

            if (use) {
                b.term.onTestStart();
            } else {
                b.term.onTestEnd();
            }
        }
    }

    private void notifyTerminatorsEnd() {
        for (TerminatorBinding b : terminators) {
            b.term.onTestEnd();
        }
    }

    public void addTerminator(DiagTerminatorInterface term) {
        addTerminator(term, term.getTerminatorName(), true);
    }

    public void addTerminator(DiagTerminatorInterface term,
                              String label,
                              boolean defaultUse) {
        if (term == null) {
            return;
        }
        if (label == null || label.isEmpty()) {
            label = term.getTerminatorName();
        }

        TerminatorBinding binding = new TerminatorBinding(term, label, defaultUse);
        terminators.add(binding);

        // If dashboard already initialized, seed the UseTerm_* entry now
        if (dashboardInitialized) {
            String key = baseKey + "UseTerm_" + label;
            SmartDashboard.putBoolean(key, defaultUse);
        }
    }

    // --------------------------------------------------------------
    // Dashboard helpers
    // --------------------------------------------------------------

    private void ensureDashboardKeys() {
        if (dashboardInitialized) {
            return;
        }
        dashboardInitialized = true;

        SmartDashboard.putBoolean(baseKey + "Enable", false);
        SmartDashboard.putBoolean(baseKey + "Retry",  false);

        for (TerminatorBinding b : terminators) {
            String key = baseKey + "UseTerm_" + b.label;
            SmartDashboard.putBoolean(key, b.defaultUse);
        }
    }

    private void updateDashboard() {
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

        String msg = DiagStatus32.getMessage(lastTestStatus);

        // If a terminator fired, append its label
        int fac = DiagStatus32.getFacility(lastTestStatus);
        if (fac == DiagStatus32.FAC_TERMINATOR && lastTerminatorLabel != null) {
            msg = msg + " (" + lastTerminatorLabel + ")";
        }

        return prefix + msg;
    }
}
