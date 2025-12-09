package frc.robot.diag.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Base class for all diagnostic devices.
 *
 * Responsibilities:
 *  - Register each device in a global list.
 *  - Manage per-device Enable and Retry controls from SmartDashboard.
 *  - Run tests only when enabled.
 *  - Stop tests when disabled or when a terminator fires.
 *  - Do a full close/re-open on Retry.
 *  - Publish health, summary, and status (decimal + hex) to the dashboard.
 *
 * Subclasses implement the hardware-specific pieces:
 *  - openHardware()
 *  - closeHardware()
 *  - runHardwareTest()
 *  - stopHardware()
 *  - (optional) onTestStart(), onTestEnd()
 */
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

    private final List<DiagTerminator> terminators = new ArrayList<>();

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

    public void addTerminator(DiagTerminator term) {
        if (term != null) {
            terminators.add(term);
        }
    }

    // ----------------------------------------------------------------
    // Hooks for subclasses
    // ----------------------------------------------------------------

    /**
     * Allocate/open hardware for this device.
     * Return a status code (S_INIT_OK, S_INIT_FAIL, or device-specific).
     */
    protected abstract int openHardware();

    /**
     * Close/deallocate hardware. Must be safe to call even if open failed.
     */
    protected abstract void closeHardware();

    /**
     * Single test step. Called repeatedly while the test is running.
     * Return a status describing the current step result.
     */
    protected abstract int runHardwareTest();

    /**
     * Stop the hardware safely (set outputs to zero, etc.).
     */
    protected abstract void stopHardware();

    /**
     * Optional hook when a test starts.
     */
    protected void onTestStart() {}

    /**
     * Optional hook when a test ends.
     */
    protected void onTestEnd() {}

    // ----------------------------------------------------------------
    // Periodic
    // ----------------------------------------------------------------

    public void periodic() {
        initDashboardIfNeeded();

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

            publish();
            return;
        }

        // Not enabled: make sure we are not running and just publish current state
        if (!enable) {
            if (running) {
                stopHardware();
                onTestEnd();
                running = false;
            }
            // Don't clear testCompleted here; disabling preserves last result.
            publish();
            return;
        }

        // Enabled but test was already completed by terminator or fatal init
        if (testCompleted) {
            publish();
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

                publish();
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

            publish();
            return;
        }

        // Normal test step
        int stepStatus = runHardwareTest();
        lastTestStatus = stepStatus;
        everRan = true;

        publish();
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
        for (DiagTerminator t : terminators) {
            int s = t.getTerminatorStatus();
            if (s != 0) {
                return s;
            }
        }
        return 0;
    }

    // ----------------------------------------------------------------
    // Dashboard helpers
    // ----------------------------------------------------------------

    private void initDashboardIfNeeded() {
        if (dashboardInitialized) {
            return;
        }
        dashboardInitialized = true;

        SmartDashboard.putBoolean(baseKey + "Enable", false);
        SmartDashboard.putBoolean(baseKey + "Retry", false);

        SmartDashboard.putString(baseKey + "Health", "UNKNOWN");
        SmartDashboard.putString(baseKey + "StatusSummary", "test not run");

        // Both decimal and hex, initialized to 0
        SmartDashboard.putString(baseKey + "LastStatusHex", "0x00000000");
    }

    private void publish() {
        publishLastStatus(lastTestStatus);
        SmartDashboard.putString(baseKey + "Health", computeHealth());
        SmartDashboard.putString(baseKey + "StatusSummary", computeSummary());
    }

    /**
     * Publish the raw status in decimal and a human-readable hex string.
     */
    private void publishLastStatus(int status) {
        SmartDashboard.putString(
                baseKey + "LastStatusHex",
                String.format("0x%08X", status));
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
            // e.g. WARNING or any mid-level stuff
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
