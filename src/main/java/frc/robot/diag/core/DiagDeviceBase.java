package frc.robot.diag.core;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DiagDeviceBase {

    // ------------------------------------------------------------
    // Global device registry
    // ------------------------------------------------------------

    private static final List<DiagDeviceBase> DEVICES = new ArrayList<>();

    public static List<DiagDeviceBase> getDevices() {
        return Collections.unmodifiableList(DEVICES);
    }

    // ------------------------------------------------------------
    // Global run control
    // ------------------------------------------------------------

    private static boolean globalsInitialized = false;

    private static boolean runActive = false;
    private static int runId = 0;

    // Latches so toggle buttons act like one-shots
    private static boolean startConsumedWhileHigh = false;
    private static boolean stopConsumedWhileHigh  = false;

    private static final String GLOBAL_START_KEY     = "Diag/StartTest";
    private static final String GLOBAL_STOP_KEY      = "Diag/StopTest";
    private static final String GLOBAL_RUNID_KEY     = "Diag/RunId";
    private static final String GLOBAL_RUNACTIVE_KEY = "Diag/RunActive";
    private static final String GLOBAL_RUNSTART_KEY  = "Diag/RunStartTimeSec";

    private static void ensureGlobalDashboardKeys() {
        if (globalsInitialized) return;
        globalsInitialized = true;

        SmartDashboard.putBoolean(GLOBAL_START_KEY, false);
        SmartDashboard.putBoolean(GLOBAL_STOP_KEY, false);
        SmartDashboard.putNumber(GLOBAL_RUNID_KEY, 0);
        SmartDashboard.putBoolean(GLOBAL_RUNACTIVE_KEY, false);
        SmartDashboard.putNumber(GLOBAL_RUNSTART_KEY, 0.0);
    }

    private static void pollGlobalRunButtons() {

        ensureGlobalDashboardKeys();

        boolean startBtn = SmartDashboard.getBoolean(GLOBAL_START_KEY, false);
        boolean stopBtn  = SmartDashboard.getBoolean(GLOBAL_STOP_KEY, false);

        // release latches when toggled low
        if (!startBtn) startConsumedWhileHigh = false;
        if (!stopBtn)  stopConsumedWhileHigh  = false;

        // StopTest = one-shot abort
        if (stopBtn && runActive && !stopConsumedWhileHigh) {

            stopConsumedWhileHigh = true;

            runActive = false;
            SmartDashboard.putBoolean(GLOBAL_RUNACTIVE_KEY, false);

            for (DiagDeviceBase d : DEVICES) {
                d.abortRunNow();
            }
        }

        // StartTest = one-shot start
        if (startBtn && !runActive && !startConsumedWhileHigh) {

            startConsumedWhileHigh = true;

            runId++;
            runActive = true;

            SmartDashboard.putNumber(GLOBAL_RUNID_KEY, runId);
            SmartDashboard.putBoolean(GLOBAL_RUNACTIVE_KEY, true);
            SmartDashboard.putNumber(GLOBAL_RUNSTART_KEY, Timer.getFPGATimestamp());

            for (DiagDeviceBase d : DEVICES) {
                d.prepareForNewRun(runId);
            }
        }
    }

    private static void autoEndRunIfAllSelectedComplete() {

        if (!runActive) return;

        boolean anySelected = false;

        for (DiagDeviceBase d : DEVICES) {

            boolean selected = d.selectedForRun && (d.activeRunId == runId);
            if (!selected) continue;

            anySelected = true;

            if (!d.testCompleted || d.running) {
                return;
            }
        }

        if (!anySelected) return;

        runActive = false;
        SmartDashboard.putBoolean(GLOBAL_RUNACTIVE_KEY, false);
    }

    public static void periodicAll() {

        pollGlobalRunButtons();

        for (DiagDeviceBase d : DEVICES) {
            d.periodic();
        }

        autoEndRunIfAllSelectedComplete();
    }

    public static void stopAll() {

        runActive = false;
        ensureGlobalDashboardKeys();
        SmartDashboard.putBoolean(GLOBAL_RUNACTIVE_KEY, false);

        for (DiagDeviceBase d : DEVICES) {
            d.stopTesting();
        }
    }

    // ------------------------------------------------------------
    // Instance state
    // ------------------------------------------------------------

    private final String diagName;
    protected final String baseKey;

    private boolean dashboardInitialized = false;

    protected boolean running = false;
    protected boolean testCompleted = false;
    protected boolean everRan = false;

    protected int lastTestStatus = DiagStatus32.S_UNSET;

    protected String lastFiredTerminatorName = "";
    protected int lastFiredTerminatorStatus = DiagStatus32.S_UNSET;

    private int activeRunId = 0;
    private boolean selectedForRun = false;

    protected static class TermBinding {
        public final DiagTerminatorInterface term;
        public final String useKey;

        public boolean armed = false;
        public boolean activeUse = false;

        public int lastTermStatus = DiagStatus32.TERM_CONTINUE;
        public String lastTermDebug = "";

        public TermBinding(DiagTerminatorInterface term, String useKey) {
            this.term = term;
            this.useKey = useKey;
        }
    }

    protected final List<TermBinding> terminators = new ArrayList<>();

    // ------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------

    protected DiagDeviceBase(String diagName) {
        this.diagName = diagName;
        this.baseKey = "Diag/" + diagName + "/";
        DEVICES.add(this);
    }

    public String getDiagName() {
        return diagName;
    }

    // ------------------------------------------------------------
    // Hardware hooks
    // ------------------------------------------------------------

    protected abstract int openHardware();
    protected abstract void closeHardware();
    protected abstract int runHardwareTest();
    protected abstract void stopHardware();

    protected void onTestStart() { }
    protected void onTestEnd() { }

    protected String getDebugStateExtra() {
        return "";
    }

    // ------------------------------------------------------------
    // Terminators
    // ------------------------------------------------------------

    public void addTerminator(DiagTerminatorInterface term, boolean defaultUse) {

        if (term == null) return;

        String useKey = baseKey + "UseTerm_" + term.getTerminatorName();
        SmartDashboard.putBoolean(useKey, defaultUse);

        terminators.add(new TermBinding(term, useKey));

        if (term instanceof DiagBindableTerminator) {
            ((DiagBindableTerminator) term).bindDut(this);
        }
    }

    public void addTerminator(DiagTerminatorInterface term) {
        addTerminator(term, false);
    }

    private void prepareForNewRun(int newRunId) {

        selectedForRun = SmartDashboard.getBoolean(baseKey + "Enable", false);
        activeRunId = newRunId;

        for (TermBinding b : terminators) {
            b.activeUse = SmartDashboard.getBoolean(b.useKey, false);
            b.armed = false;
            b.lastTermStatus = DiagStatus32.TERM_CONTINUE;
            b.lastTermDebug = "";
        }

        if (selectedForRun) {
            testCompleted = false;
            lastFiredTerminatorName = "";
            lastFiredTerminatorStatus = DiagStatus32.S_UNSET;
        }
    }

    private void abortRunNow() {

        disarmAllTerminators();

        if (running) {
            stopHardware();
            onTestEnd();
        }

        running = false;
        testCompleted = true;
    }

    private int checkTerminatorsActive() {

        for (TermBinding b : terminators) {

            if (!b.activeUse || !b.armed) continue;

            int s = b.term.getTerminatorStatus();
            b.lastTermStatus = s;

            b.lastTermDebug = b.term.getTerminatorDebug();

            if (s != DiagStatus32.TERM_CONTINUE) {
                lastFiredTerminatorName = b.term.getTerminatorName();
                lastFiredTerminatorStatus = s;
                return s;
            }
        }

        return DiagStatus32.TERM_CONTINUE;
    }

    private void syncTerminatorsArmingActive() {

        for (TermBinding b : terminators) {

            if (b.activeUse && !b.armed) {
                b.term.armForTest();
                b.armed = true;
            }

            if (!b.activeUse && b.armed) {
                b.term.disarmForTest();
                b.armed = false;
            }
        }
    }

    protected void disarmAllTerminators() {
        for (TermBinding b : terminators) {
            if (b.armed) {
                b.term.disarmForTest();
                b.armed = false;
            }
        }
    }

    // ------------------------------------------------------------
    // Main per-device tick
    // ------------------------------------------------------------

    void periodic() {

        ensureGlobalDashboardKeys();

        if (!dashboardInitialized) {
            updateDashboard(false, false);
            return;
        }

        boolean stagedEnable = SmartDashboard.getBoolean(baseKey + "Enable", false);
        boolean retry        = SmartDashboard.getBoolean(baseKey + "Retry", false);

        updateDashboard(stagedEnable, retry);

        if (retry) {

            disarmAllTerminators();
            stopHardware();
            closeHardware();

            running = false;
            testCompleted = false;
            everRan = false;
            lastTestStatus = DiagStatus32.S_UNSET;

            SmartDashboard.putBoolean(baseKey + "Retry", false);
            return;
        }

        if (!runActive) {

            if (running) {
                disarmAllTerminators();
                stopHardware();
                onTestEnd();
                running = false;
            }

            return;
        }

        if (!selectedForRun || activeRunId != runId) return;

        if (!running && !testCompleted) {

            int s = openHardware();

            if (DiagStatus32.getSeverity(s) >= DiagStatus32.SEV_ERROR) {
                lastTestStatus = s;
                testCompleted = true;
                closeHardware();
                return;
            }

            onTestStart();
            running = true;
        }

        if (!running) return;

        syncTerminatorsArmingActive();

        int termStatus = checkTerminatorsActive();
        if (termStatus != DiagStatus32.TERM_CONTINUE) {

            lastTestStatus = termStatus;

            disarmAllTerminators();
            stopHardware();
            onTestEnd();

            running = false;
            testCompleted = true;
            return;
        }

        int stepStatus = runHardwareTest();

        lastTestStatus = stepStatus;
        everRan = true;

        // FIX: error during test step is terminal
        if (DiagStatus32.getSeverity(stepStatus) >= DiagStatus32.SEV_ERROR) {

            disarmAllTerminators();
            stopHardware();
            onTestEnd();

            running = false;
            testCompleted = true;
        }
    }

    public void requestTerminateFromTerminator(String termName, int termStatus) {

        if (!running) return;

        lastFiredTerminatorName = termName;
        lastFiredTerminatorStatus = termStatus;
        lastTestStatus = termStatus;

        disarmAllTerminators();
        stopHardware();
        onTestEnd();

        running = false;
        testCompleted = true;
    }

    public void stopTesting() {

        disarmAllTerminators();
        stopHardware();
        onTestEnd();

        running = false;
        testCompleted = false;
    }

    // ------------------------------------------------------------
    // Dashboard
    // ------------------------------------------------------------

    void updateDashboard(boolean enable, boolean retry) {

        if (!dashboardInitialized) {

            dashboardInitialized = true;

            SmartDashboard.putBoolean(baseKey + "Enable", false);
            SmartDashboard.putBoolean(baseKey + "Retry", false);

            SmartDashboard.putString(baseKey + "LastStatusHex",
                    String.format("0x%08X", DiagStatus32.S_UNSET));
            SmartDashboard.putString(baseKey + "Health", "UNKNOWN");
            SmartDashboard.putString(baseKey + "StatusSummary", "test not run");
            SmartDashboard.putString(baseKey + "State", "IDLE");
            SmartDashboard.putString(baseKey + "DebugState", "init");

            for (TermBinding b : terminators) {
                SmartDashboard.putBoolean(b.useKey, false);
            }
        }

        SmartDashboard.putBoolean(baseKey + "Enable", enable);
        SmartDashboard.putBoolean(baseKey + "Retry", retry);

        SmartDashboard.putString(baseKey + "LastStatusHex",
                String.format("0x%08X", lastTestStatus));

        int sev = DiagStatus32.getSeverity(lastTestStatus);

        String health =
                (sev < DiagStatus32.SEV_WARNING) ? "GOOD" :
                (sev >= DiagStatus32.SEV_ERROR) ? "ERROR" : "WARNING";

        SmartDashboard.putString(baseKey + "Health", health);

        String msg = DiagStatus32.getMessage(lastTestStatus);
        SmartDashboard.putString(baseKey + "StatusSummary", msg);

        String state =
                running ? "TESTING" :
                testCompleted ? "DONE" : "IDLE";

        SmartDashboard.putString(baseKey + "State", state);

        SmartDashboard.putString(baseKey + "DebugState",
                "runActive=" + (runActive ? 1 : 0) +
                " runId=" + runId +
                " running=" + (running ? 1 : 0) +
                " completed=" + (testCompleted ? 1 : 0));
    }
}
