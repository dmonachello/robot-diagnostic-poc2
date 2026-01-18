package frc.robot.diag.devices;

import java.lang.reflect.Field;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.diag.core.DiagDeviceBase;
import frc.robot.diag.core.DiagPositionProvider;
import frc.robot.diag.core.DiagStatus32;
import frc.robot.diag.core.TerminatorDeviceBase;

/**
 * Rotation (delta) terminator.
 *
 * Behavior:
 * - On each new test run (Diag/RunId changes), snapshot each bound DUT's start position.
 * - Triggers when ANY bound DUT has moved by deltaRotations (absolute delta) from its snapshot.
 * - TerminatorDeviceBase will stop ALL bound DUTs immediately when this triggers.
 *
 * If you want different deltas per motor, create separate instances.
 */
public class RotationTerminator extends TerminatorDeviceBase {

    public static final int TERM_ROT_REACHED = DiagStatus32.defineStatus(
            DiagStatus32.CTRL_NONE,
            DiagStatus32.FAC_TERMINATOR,
            120,
            DiagStatus32.SEV_SUCCESS,
            "rotation delta reached"
    );

    private static final String GLOBAL_RUNID_KEY = "Diag/RunId";

    private final double deltaRotations;

    private final Map<DiagDeviceBase, Double> startPos = new IdentityHashMap<>();

    private int lastRunIdSeen = -1;

    private String firedByName = "";
    private double firedDelta = 0.0;

    // Reflection hook into TerminatorDeviceBase binding list
    private Field boundListField = null;

    public RotationTerminator(String diagName, double deltaRotations) {
        super(diagName);
        this.deltaRotations = Math.abs(deltaRotations);
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

    private void resnapshotIfNewRun(List<DiagDeviceBase> duts) {

        int runId = (int) SmartDashboard.getNumber(GLOBAL_RUNID_KEY, 0);

        if (runId != lastRunIdSeen) {
            lastRunIdSeen = runId;

            startPos.clear();
            firedByName = "";
            firedDelta = 0.0;

            if (duts == null) return;

            for (DiagDeviceBase d : duts) {
                if (d instanceof DiagPositionProvider) {
                    double p0 = ((DiagPositionProvider) d).getPositionRotations();
                    startPos.put(d, p0);
                }
            }
        }
    }

    @Override
    protected int evalTerminatorStatus() {

        List<DiagDeviceBase> duts = getBoundDutsViaReflection();
        if (duts == null || duts.isEmpty()) {
            return DiagStatus32.TERM_CONTINUE;
        }

        resnapshotIfNewRun(duts);

        for (DiagDeviceBase d : duts) {

            Double p0 = startPos.get(d);
            if (p0 == null) continue;

            if (!(d instanceof DiagPositionProvider)) continue;

            double p = ((DiagPositionProvider) d).getPositionRotations();
            double delta = Math.abs(p - p0.doubleValue());

            if (delta >= deltaRotations) {
                firedByName = d.getDiagName();
                firedDelta = delta;
                return TERM_ROT_REACHED;
            }
        }

        return DiagStatus32.TERM_CONTINUE;
    }

    @Override
    public String getTerminatorDebug() {
        if (firedByName == null || firedByName.isEmpty()) {
            return "deltaRot=" + String.format("%.2f", deltaRotations)
                    + " runId=" + lastRunIdSeen;
        }
        return "deltaRot=" + String.format("%.2f", deltaRotations)
                + " hitBy=" + firedByName
                + " d=" + String.format("%.2f", firedDelta)
                + " runId=" + lastRunIdSeen;
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
