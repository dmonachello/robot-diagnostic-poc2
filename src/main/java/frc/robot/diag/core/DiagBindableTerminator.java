package frc.robot.diag.core;

/**
 * Optional interface for terminators that want to know which DUTs they are bound to.
 * Used for scheme 1 shared-terminator behavior without changing Robot.java.
 */
public interface DiagBindableTerminator {
    void bindDut(DiagDeviceBase dut);
}
