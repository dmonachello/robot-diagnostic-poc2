package frc.robot.diag.core;

public abstract class MotorDiagDeviceBase extends DiagDeviceBase {

    protected final double testDuty;

    protected MotorDiagDeviceBase(String name, double testDuty) {
        super(name);
        this.testDuty = testDuty;
    }
}
