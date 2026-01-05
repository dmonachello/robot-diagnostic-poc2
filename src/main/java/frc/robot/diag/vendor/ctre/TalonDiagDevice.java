package frc.robot.diag.vendor.ctre;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.diag.core.DiagDeviceBase;

/*
 * Diagnostic wrapper for a single TalonFX motor.
 *
 * Semantics match SparkDiagDevice:
 * - openHardware(): construct TalonFX, clear state
 * - runHardwareTest(): command a small duty and require that position changes
 *   by at least MIN_ROTATIONS_OK within MAX_STEPS cycles
 * - stopHardware(): stop motor, but do NOT clear failure latch
 * - closeHardware(): stop + close, clear all state
 *
 * Once a test has failed, this device keeps reporting error until re-opened.
 */
public class TalonDiagDevice extends DiagDeviceBase {

    private final int    canId;
    private final double testDuty;

    private TalonFX motor;
    private DutyCycleOut duty;

    // Tunables (kept same as Spark)
    private static final double MIN_ROTATIONS_OK = 0.25;
    private static final int    MIN_STEPS_BEFORE_JUDGE = 10;
    private static final int    MAX_STEPS = 50;

    // Per-test state
    private boolean testActive = false;
    private boolean testFailed = false;
    private double  startPosition = 0.0;
    private double  maxDelta = 0.0;
    private int     stepCount = 0;

    public TalonDiagDevice(String diagName, int canId, double testDuty) {
        super(diagName);
        this.canId = canId;
        this.testDuty = testDuty;
        this.motor = null;
        this.duty = new DutyCycleOut(0.0);
    }

    @Override
    protected int openHardware() {
        try {
            closeHardwareInternal();
            motor = new TalonFX(canId);
            resetTestState();
            return TalonDiagStatus.S_OPEN_OK;
        } catch (Exception e) {
            motor = null;
            resetTestState();
            return TalonDiagStatus.S_OPEN_FATAL;
        }
    }

    @Override
    protected void closeHardware() {
        closeHardwareInternal();
    }

    private void closeHardwareInternal() {
        if (motor != null) {
            try {
                duty.Output = 0.0;
                motor.setControl(duty);
                motor.close();
            } catch (Exception ignored) {
            }
            motor = null;
        }
        resetTestState();
    }

    private void resetTestState() {
        testActive = false;
        testFailed = false;
        startPosition = 0.0;
        maxDelta = 0.0;
        stepCount = 0;
    }

    @Override
    protected int runHardwareTest() {
        if (motor == null) {
            testFailed = true;
            return TalonDiagStatus.S_CMD_ERROR;
        }

        if (testFailed) {
            try {
                duty.Output = 0.0;
                motor.setControl(duty);
            } catch (Exception ignored) {
            }
            return TalonDiagStatus.S_CMD_ERROR;
        }

        // First call after Enable turned on: initialize baseline
        if (!testActive) {
            startPosition = safeGetPosition();
            maxDelta = 0.0;
            stepCount = 0;
            testActive = true;

            try {
                duty.Output = testDuty;
                motor.setControl(duty);
            } catch (Exception e) {
                testFailed = true;
                return TalonDiagStatus.S_CMD_ERROR;
            }

            return TalonDiagStatus.S_CMD_OK;
        }

        // Normal test step: drive and measure motion
        try {
            duty.Output = testDuty;
            motor.setControl(duty);
        } catch (Exception e) {
            testFailed = true;
            return TalonDiagStatus.S_CMD_ERROR;
        }

        double pos = safeGetPosition();
        double delta = Math.abs(pos - startPosition);

        if (delta > maxDelta) {
            maxDelta = delta;
        }
        stepCount++;

        if (stepCount < MIN_STEPS_BEFORE_JUDGE) {
            return TalonDiagStatus.S_CMD_OK;
        }

        if (maxDelta >= MIN_ROTATIONS_OK) {
            return TalonDiagStatus.S_CMD_OK;
        }

        if (stepCount >= MAX_STEPS) {
            testFailed = true;
            return TalonDiagStatus.S_CMD_ERROR;
        }

        return TalonDiagStatus.S_CMD_OK;
    }

    @Override
    protected void stopHardware() {
        testActive = false;
        // IMPORTANT: do NOT clear testFailed here (same as Spark)
        if (motor != null) {
            try {
                duty.Output = 0.0;
                motor.setControl(duty);
            } catch (Exception ignored) {
            }
        }
    }

    private double safeGetPosition() {
        try {
            var sig = motor.getPosition();
            // Force a refresh so we’re not staring at stale data
            sig.refresh();

            // If the signal is unhealthy, treat like "no movement"
            if (!sig.getStatus().isOK()) {
                return 0.0;
            }
            return sig.getValueAsDouble();
        } catch (Exception e) {
            return 0.0;
        }
    }
}
