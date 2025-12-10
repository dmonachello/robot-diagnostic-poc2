package frc.robot.diag.core;

/**
 * Simple per-device status holder.
 * No SmartDashboard / NetworkTables here.
 */
public class DiagDeviceStatus {

    // Last status code for this device (DiagStatus32 int)
    private int lastStatus = 0;

    public DiagDeviceStatus() {
    }

    public void setLastStatus(int status) {
        this.lastStatus = status;
    }

    public int getLastStatus() {
        return lastStatus;
    }

    public String getLastStatusHex() {
        return String.format("0x%08X", lastStatus);
    }
}
