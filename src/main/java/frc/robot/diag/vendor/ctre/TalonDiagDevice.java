package frc.robot.diag.vendor.ctre;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.core.DiagStatus32;

/**
 * Diagnostic wrapper for a single TalonFX motor.
 *
 * Key rules for synchronization:
 * - TalonFX object is created once (not per test run).
 * - openHardware() does NOT touch CAN or block.
 * - First motor command is issued in onTestStart().
 * - runHardwareTest() only maintains output and samples last-known values.
 */
public class TalonDiagDevice extends DiagDeviceBase {

    private final int    canId;
    private final double testDuty;

    private final TalonFX motor;
    private final DutyCycleOut duty;

    // last-known values for dashboard/debug only
    private double lastVelRps = 0.0;
    private double lastOutPct = 0.0;
    private double lastBusV   = 0.0;

    public TalonDiagDevice(String diagName, int canId, double testDuty) {
        super(diagName);
        this.canId = canId;
        this.testDuty = testDuty;

        duty = new DutyCycleOut(0.0);
        duty.EnableFOC = false;

        // IMPORTANT:
        // Create the TalonFX once, outside the test timing path.
        motor = new TalonFX(canId);
    }

    @Override
    protected int openHardware() {
        // Hardware already exists; nothing to block or wait on.
        return TalonDiagStatus.S_OPEN_OK;
    }

    @Override
    protected void closeHardware() {
        // Do not destroy the TalonFX here.
        // We keep it alive across runs to avoid CAN rediscovery delays.
    }

    @Override
    protected void onTestStart() {
        // Issue first command immediately when the test starts.
        duty.Output = testDuty;
        motor.setControl(duty);
    }

    @Override
    protected int runHardwareTest() {

        try {
            // Maintain output (safe to repeat every tick)
            duty.Output = testDuty;
            motor.setControl(duty);

            // Non-blocking reads of last-known values
            lastVelRps = motor.getRotorVelocity().getValue()
                    .in(Units.RotationsPerSecond);
            lastOutPct = motor.getDutyCycle().getValue();
            lastBusV   = motor.getSupplyVoltage().getValue()
                    .in(Units.Volts);

            return TalonDiagStatus.S_CMD_OK;

        } catch (Exception e) {
            return TalonDiagStatus.S_CMD_ERROR;
        }
    }

    @Override
    protected void stopHardware() {
        try {
            duty.Output = 0.0;
            motor.setControl(duty);
        } catch (Exception ignored) {
        }
    }

    @Override
    protected String getDebugStateExtra() {
        return " talon(canId=" + canId
                + " velRps=" + String.format("%.2f", lastVelRps)
                + " out=" + String.format("%.2f", lastOutPct)
                + " busV=" + String.format("%.1f", lastBusV)
                + ")";
    }
}
