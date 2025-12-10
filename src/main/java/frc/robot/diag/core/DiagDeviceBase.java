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

    // Per-device status holder: just lastStatus + hex formatter
    protected final DiagDeviceStatus status = new DiagDeviceStatus();

    private final List<DiagTerminatorInterface> terminators = new ArrayList<>();

    // Status tracking
    private int lastInitStatus = 0;
    private int lastTestStatus = 0;
    private boolean everRan    = false;

    // Runtime state
    private boolean running       = false;
    private boolean testCompleted = false;   // true after terminator or fatal init

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

    protected void onTestStart() {}
    protected void onTestEnd() {}

    // ----------------------------------------------------------------
    // Periodic
    // ----------------------------------------------------------------

    public void periodic() {
        updateDashboard();

        boolean enable = SmartDashboard.getBoolean(baseKey + "Enable", false);
        boolean retry  = SmartDashboard.getBoolean(baseKey + "Retry", false);

        // Retry: full reset and re-open on next enabled cycle
        if (retry) {
            SmartDashboard.putBoolean(baseKey + "Retry", false);

            if (running) {
                stopHardware();
                onTestEnd();
                running = false;
            }
            closeHardware();

            lastInitStatus  = 0;
            lastTestStatus  = 0;
            everRan         = false;
            testCompleted   = false;

            updateDashboard();
            return;
        }

        // Not enabled: make sure we are not running and just publish current state
        if (!enable) {
            if (running) {
                stopHardware();
                onTestEnd();
                running = false;
            }
            // disabling preserves last result
            updateDashboard();
            return;
        }

        // Enabled but test was already completed by terminator or fatal init
        if (testCompleted) {
            updateDashboard();
            return;
        }

        // Start test if not running yet
        if (!running) {
            lastInitStatus = openHardware();
            running = true;
            onTestStart();

            // If init itself was an error/fatal, treat as completed test
            if (DiagStatus32.getSeverity(lastInitStatus) >= DiagStatus32.SEV_ERROR) {
                lastTestStatus = lastInitStatus;
                everRan = true;

                stopHardware();
                onTestEnd();
                running = false;
                testCompleted = true;

                updateDashboard();
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
            return;
        }

        // Normal test step
        int stepStatus = runHardwareTest();
        lastTestStatus = stepStatus;
        everRan = true;

        updateDashboard();
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
        for (DiagTerminatorInterface t : terminators) {
            int s = t.getTerminatorStatus();
            if (s != 0) {
                return s;
            }
        }
        return 0;
    }

    public void addTerminator(DiagTerminatorInterface term) {
        if (term != null) {
            terminators.add(term);
        }
    }


    // ----------------------------------------------------------------
    // Dashboard helpers
    // ----------------------------------------------------------------

    private void updateDashboard() {
        // One-time setup
        if (!dashboardInitialized) {
            dashboardInitialized = true;
    
            // Set up the controls once. We never touch these again here.
            SmartDashboard.putBoolean(baseKey + "Enable", false);
            SmartDashboard.putBoolean(baseKey + "Retry",  false);
    
            // You could also seed initial text here if you want,
            // but it will get overwritten by the logic below anyway.
        }
    
        // Live status every time
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

        return prefix + DiagStatus32.getMessage(lastTestStatus);
    }
}
