package frc.robot.diag.core;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DiagDeviceStatus {

    // Only the hex form of the status code is kept
    private String lastStatusHex = "";

    // Per-device test counter
    private int testsRun = 0;

    public DiagDeviceStatus() {
    }

    // Caller gives raw int status, we store hex text
    public void setLastStatus(int status) {
        lastStatusHex = String.format("0x%08X", status);
    }

    public String getLastStatusHex() {
        return lastStatusHex;
    }

    // Increment every time this device runs a test
    public void incrementTestsRun() {
        testsRun++;
    }

    // Clear the test counter
    public void resetCounters() {
        testsRun = 0;
    }

    public int getTestsRun() {
        return testsRun;
    }

    // Dashboard publishing
    public void publishToDashboard(NetworkTable table) {
        table.getEntry("lastStatusHex").setString(lastStatusHex);
        table.getEntry("testsRun").setNumber(testsRun);
    }

    public void publishToDashboard(String tablePath) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(tablePath);
        publishToDashboard(table);
    }
}
