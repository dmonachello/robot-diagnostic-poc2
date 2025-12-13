package frc.robot.diag.core;

public class DiagDeviceStatus {

    private int lastStatus = DiagStatus32.S_UNSET;

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
