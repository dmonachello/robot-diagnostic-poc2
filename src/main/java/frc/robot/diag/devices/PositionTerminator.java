package frc.robot.diag.devices;

import java.lang.reflect.Field;
import java.util.List;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.core.DiagPositionProvider;
import frc.robot.diag.core.DiagStatus32;
import frc.robot.diag.core.TerminatorDeviceBase;

/**
 * Absolute position terminator.
 *
 * Behavior:
 * - Triggers when ANY bound DUT reaches abs(position) >= limitRotations.
 * - Does NOT resnapshot per run. Once you're at/over the limit, future runs will
 *   immediately terminate until the mechanism is moved back under the limit.
 *
 * Units: rotations (motor/rotor rotations).
 */
public class PositionTerminator extends TerminatorDeviceBase {

    public static final int TERM_POS_LIMIT = DiagStatus32.defineStatus(
            DiagStatus32.CTRL_NONE,
            DiagStatus32.FAC_TERMINATOR,
            121,
            DiagStatus32.SEV_WARNING,
            "position limit reached"
    );

    private final double limitRotations;

    private String firedByName = "";
    private double firedPos = 0.0;

    private Field boundListField = null;

    public PositionTerminator(String diagName, double limitRotations) {
        super(diagName);
        this.limitRotations = Math.abs(limitRotations);
        initReflection();
    }

    private void initReflection() {

        for (Field f : TerminatorDeviceBase.class.getDeclaredFields()) {
            if (!List.class.isAssignableFrom(f.getType())) continue;
            String n = f.getName().toLowerCase();
            if (n.contains("dut") || n.contains("bound") || n.contains("device")) {
                f.setAccessible(true);
                boundListField = f;
                break;
            }
        }

        if (boundListField == null) {
            for (Field f : TerminatorDeviceBase.class.getDeclaredFields()) {
                if (List.class.isAssignableFrom(f.getType())) {
                    f.setAccessible(true);
                    boundListField = f;
                    break;
                }
            }
        }
    }

    @SuppressWarnings("unchecked")
    private List<DiagDeviceBase> getBoundDutsViaReflection() {
        if (boundListField == null) return null;
        try {
            Object v = boundListField.get(this);
            if (v instanceof List) {
                return (List<DiagDeviceBase>) v;
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    @Override
    protected int evalTerminatorStatus() {

        List<DiagDeviceBase> duts = getBoundDutsViaReflection();
        if (duts == null || duts.isEmpty()) {
            return DiagStatus32.TERM_CONTINUE;
        }

        for (DiagDeviceBase d : duts) {

            if (!(d instanceof DiagPositionProvider)) continue;

            double p = ((DiagPositionProvider) d).getPositionRotations();
            double ap = Math.abs(p);

            if (ap >= limitRotations) {
                firedByName = d.getDiagName();
                firedPos = p;
                return TERM_POS_LIMIT;
            }
        }

        return DiagStatus32.TERM_CONTINUE;
    }

    @Override
    public String getTerminatorDebug() {
        if (firedByName == null || firedByName.isEmpty()) {
            return "absPosLimit=" + String.format("%.2f", limitRotations);
        }
        return "absPosLimit=" + String.format("%.2f", limitRotations)
                + " hitBy=" + firedByName
                + " pos=" + String.format("%.2f", firedPos);
    }

    @Override
    protected int openHardware() {
        return DiagStatus32.S_INIT_OK;
    }

    @Override
    protected void closeHardware() {
    }

    @Override
    protected int runHardwareTest() {
        return DiagStatus32.S_TEST_OK;
    }

    @Override
    protected void stopHardware() {
    }
}
