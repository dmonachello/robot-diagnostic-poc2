package frc.robot.diag.vendor.rev;

// import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.core.DiagPositionProvider;
//import frc.robot.diag.core.DiagStatus32;

/**
 * Diagnostic device for a SparkMax + NEO combination.
 *
 * Semantics:
 * - openHardware(): construct SparkMax and get its RelativeEncoder.
 * - runHardwareTest(): drive the motor at a small duty and require that
 *   the encoder position changes by at least MIN_ROTATIONS_OK within
 *   MAX_STEPS cycles. If not, we latch a failure for this device.
 * - stopHardware(): stop the motor, but do NOT clear the failure latch.
 * - closeHardware(): stop and close the Spark, and clear all state.
 *
 * Once a test has failed (encoder never moved enough), this device will
 * keep reporting an error status on further test calls until it is
 * re-opened (e.g. after a Retry).
 */
public class SparkDiagDevice extends DiagDeviceBase implements DiagPositionProvider {

    private final int canId;
    private final MotorType motorType;
    private final double testDuty;

    private SparkMax motor;
    private RelativeEncoder encoder;

    // Per-test state
    private boolean testActive = false;
    private boolean testFailed = false;
    private double startPosition = 0.0;
    private double maxDelta = 0.0;
    private int stepCount = 0;

    // Tunables for the NEO verification test
    private static final double MIN_ROTATIONS_OK = 0.25;  // must move at least 1/4 turn
    private static final int MIN_STEPS_BEFORE_JUDGE = 10; // don't judge immediately
    private static final int MAX_STEPS = 50;              // cap how long we wait

    public SparkDiagDevice(String diagName, int canId, MotorType motorType, double testDuty) {
        super(diagName);
        this.canId = canId;
        this.motorType = motorType;
        this.testDuty = testDuty;
        this.motor = null;
    }

    // --------------------------------------------------------------------
    // Lifecycle hooks from DiagDeviceBase
    // --------------------------------------------------------------------

    @Override
    protected int openHardware() {
        try {
            // System.out.println("1 before open SparkMax " + motor + " can ID " + canId);
            // Fully reset any previous hardware and state
            closeHardwareInternal();
            // System.out.println("2 before open SparkMax " + motor + " can ID " + canId);

            motor = new SparkMax(canId, motorType);
            // System.out.println("after open SparkMax " + motor + " can ID " + canId);
            encoder = motor.getEncoder();  // NEO hall-sensor encoder

            resetTestState();

            return SparkDiagStatus.S_OPEN_OK;
        } catch (Exception e) {
            motor = null;
            encoder = null;
            resetTestState();
            return SparkDiagStatus.S_OPEN_FATAL;
        }
    }

    @Override
    protected void closeHardware() {
        closeHardwareInternal();
    }

    private void closeHardwareInternal() {

        if (motor != null) {
            try {
                motor.set(0.0);
            } catch (Exception ignored) {
            }
            try {
                motor.close();
            } catch (Exception ignored) {
            }
        }
        System.out.println("close hardware internal");
        motor = null;
        encoder = null;
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
        // If we never opened successfully, we can't test this device
        if (motor == null || encoder == null) {
            testFailed = true;
            // System.out.println("sp1");
            return SparkDiagStatus.S_CMD_ERROR;  // treat as hardware error            
        }

        // If we already decided this device failed, keep reporting error
        if (testFailed) {
            // Make sure we are not driving a "dead" channel forever
            try {
                motor.set(0.0);
            } catch (Exception ignored) {
            }
            // System.out.println("sp2");
            return SparkDiagStatus.S_CMD_ERROR;
        }

        // First call after Enable turned on: initialize baseline
        if (!testActive) {
            startPosition = safeGetPosition();
            maxDelta = 0.0;
            stepCount = 0;
            testActive = true;
            

            // Start driving immediately
            try {
                motor.set(testDuty);
            } catch (Exception e) {
                testFailed = true;
                // System.out.println("sp3");
                return SparkDiagStatus.S_CMD_ERROR;
            }

            // Don't mark as "good" yet – just "command sent"
            // System.out.println("sp good1");
            return SparkDiagStatus.S_CMD_OK;
        }

        // Normal test step: drive motor and measure movement
        try {
            motor.set(testDuty);
        } catch (Exception e) {
            testFailed = true;
            // System.out.println("sp4");
            return SparkDiagStatus.S_CMD_ERROR;
        }

        double pos = safeGetPosition();
        double delta = Math.abs(pos - startPosition);

        if (delta > maxDelta) {
            maxDelta = delta;
        }
        stepCount++;

        // Don't judge too early; give it a few cycles to spin
        if (stepCount < MIN_STEPS_BEFORE_JUDGE) {
            return SparkDiagStatus.S_CMD_OK;
        }

        // If we've achieved enough rotation, consider the test passed
        if (maxDelta >= MIN_ROTATIONS_OK) {
            // NOTE: We deliberately do NOT clear testActive here.
            // The base class will stop calling us when the operator
            // disables the test or a terminator fires.
            return SparkDiagStatus.S_CMD_OK;  // "command + verify OK"
        }

        // If we've run long enough and still no meaningful motion, latch failure
        if (stepCount >= MAX_STEPS) {
            testFailed = true;
            // System.out.println("sp5");
            return SparkDiagStatus.S_CMD_ERROR;  // Spark+NEO did not move
        }

        // Still running, not yet ready to declare failure
        return SparkDiagStatus.S_CMD_OK;
    }

    @Override
    protected void stopHardware() {
        testActive = false;
        // IMPORTANT: we do NOT clear testFailed here.
        // A failed device stays failed until re-opened.
        if (motor != null) {
            try {
                motor.set(0.0);
            } catch (Exception ignored) {
            }
        }
    }

    // --------------------------------------------------------------------
    // Helpers
    // --------------------------------------------------------------------

    private double safeGetPosition() {
        try {
            return (encoder != null) ? encoder.getPosition() : 0.0;
        } catch (Exception e) {
            // If encoder read blows up, treat as "no movement"
            return 0.0;
        }
    }

    @Override
    public double getPositionRotations() {
        return safeGetPosition();
    }
}
