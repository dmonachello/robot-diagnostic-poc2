package frc.robot.diag.core;

/**
 * Optional capability for DUTs: provide a position reading in rotations.
 *
 * - Units are rotations (not radians).
 * - Should be monotonic for the duration of a short test.
 * - If a device cannot provide position, it should NOT implement this interface.
 */
public interface DiagPositionProvider {
    double getPositionRotations();
}
