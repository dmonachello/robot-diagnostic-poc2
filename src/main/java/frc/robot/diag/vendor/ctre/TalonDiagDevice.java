package frc.robot.diag.vendor.ctre;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.core.DiagStatus32;

/**
 * Diagnostic wrapper for a single TalonFX motor.
 */
public class TalonDiagDevice extends DiagDeviceBase {

    private final int    canId;
    private final double testDuty;

    private TalonFX     motor;
    private DutyCycleOut duty = new DutyCycleOut(0.0);

    public TalonDiagDevice(String diagName, int canId, double testDuty) {
        super(diagName);
        this.canId    = canId;
        this.testDuty = testDuty;
    }

    @Override
    protected int openHardware() {
        try {
            closeHardware();
            motor = new TalonFX(canId);
            return TalonDiagStatus.S_OPEN_OK;
        } catch (Exception e) {
            motor = null;
            return TalonDiagStatus.S_OPEN_FATAL;
        }
    }

    @Override
    protected void closeHardware() {
        if (motor != null) {
            try {
                duty.Output = 0.0;
                motor.setControl(duty);
                motor.close();
            } catch (Exception ignored) {}
            motor = null;
        }
    }

    @Override
    protected int runHardwareTest() {
        if (motor == null) {
            return DiagStatus32.S_HW_NOT_PRESENT;
        }

        try {
            duty.Output = testDuty;
            motor.setControl(duty);

            // Basic health check using a status signal
            var tempStatus = motor.getDeviceTemp().getStatus();
            if (tempStatus.isOK()) {
                return TalonDiagStatus.S_CMD_OK;
            } else {
                return TalonDiagStatus.S_CMD_ERROR;
            }
        } catch (Exception e) {
            return TalonDiagStatus.S_CMD_ERROR;
        }
    }

    @Override
    protected void stopHardware() {
        if (motor != null) {
            try {
                duty.Output = 0.0;
                motor.setControl(duty);
            } catch (Exception ignored) {}
        }
    }
}
