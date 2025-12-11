package frc.robot.diag.core;

import java.util.HashMap;
import java.util.Map;

/**
 * VMS-style 32-bit status code helper.
 *
 * Layout (from LSB to MSB):
 *   bits  0-2 : severity (3 bits, 0..7)
 *   bits  3-15: code     (13 bits)
 *   bits 16-27: facility (12 bits)
 *   bits 28-31: ctrl     (4 bits)
 */
public final class DiagStatus32 {

    // Bit positions
    public static final int SEVERITY_SHIFT  = 0;   // 3 bits
    public static final int CODE_SHIFT      = 3;   // 13 bits
    public static final int FACILITY_SHIFT  = 16;  // 12 bits
    public static final int CTRL_SHIFT      = 28;  // 4 bits

    // Masks for unshifted values
    public static final int SEVERITY_MASK   = 0b111;    // 3 bits (0..7)
    public static final int CODE_MASK       = 0x1FFF;   // 13 bits
    public static final int FACILITY_MASK   = 0x0FFF;   // 12 bits
    public static final int CTRL_MASK       = 0xF;      // 4 bits

    // Severities
    public static final int SEV__SUCCESS = 0;
    public static final int SEV__INFO    = 1;
    public static final int SEV__WARNING = 2;
    public static final int SEV__ERROR   = 3;
    public static final int SEV__FATAL   = 4;

    public static final int SEV_SUCCESS = SEV__SUCCESS;
    public static final int SEV_INFO    = SEV__INFO;
    public static final int SEV_WARNING = SEV__WARNING;
    public static final int SEV_ERROR   = SEV__ERROR;
    public static final int SEV_FATAL   = SEV__FATAL;

    // Facilities
    public static final int FAC_GENERIC    = 0;
    public static final int FAC_TERMINATOR = 100;   // keep away from Spark/Talon/LimitSwitch

    public static final int CTRL_NONE = 0;

    // Global mapping from full 32-bit status -> human-readable text
    private static final Map<Integer, String> STATUS_TEXT = new HashMap<>();

    // ---------- Core helpers ----------

    public static int make(int ctrl, int facility, int code, int severity) {
        return  ((ctrl     & CTRL_MASK)     << CTRL_SHIFT)
              | ((facility & FACILITY_MASK) << FACILITY_SHIFT)
              | ((code     & CODE_MASK)     << CODE_SHIFT)
              | ((severity & SEVERITY_MASK) << SEVERITY_SHIFT);
    }

    public static int makeStatus(int severity, int facility, int code, int ctrl) {
        return make(ctrl, facility, code, severity);
    }

    public static int defineStatus(
            int ctrl,
            int facility,
            int code,
            int severity,
            String message) {

        int status = make(ctrl, facility, code, severity);
        STATUS_TEXT.put(status, message);
        return status;
    }

    // ---------- Extractors ----------

    public static int getSeverity(int status) {
        return (status >> SEVERITY_SHIFT) & SEVERITY_MASK;
    }

    public static int getCode(int status) {
        return (status >> CODE_SHIFT) & CODE_MASK;
    }

    public static int getFacility(int status) {
        return (status >> FACILITY_SHIFT) & FACILITY_MASK;
    }

    public static int getCtrl(int status) {
        return (status >> CTRL_SHIFT) & CTRL_MASK;
    }

    public static boolean isHigherSeverity(int a, int b) {
        return getSeverity(a) > getSeverity(b);
    }

    // ---------- Text lookup ----------

    public static String getMessage(int status) {
        String msg = STATUS_TEXT.get(status);
        if (msg != null) {
            return msg;
        }

        int sev  = getSeverity(status);
        int fac  = getFacility(status);
        int code = getCode(status);

        return "fac=" + fac + " code=" + code + " sev=" + sev
               + " (0x" + Integer.toHexString(status) + ")";
    }

    // ---------- Generic framework codes ----------

    public static final int CODE_INIT_OK        = 1;
    public static final int CODE_INIT_FAIL      = 2;
    public static final int CODE_HW_NOT_PRESENT = 3;
    public static final int CODE_TEST_OK        = 4;
    public static final int CODE_HW_FAULT       = 5;
    public static final int CODE_TERMINATED_OK  = 6;
    public static final int CODE_TERMINATED_BAD = 7;

    public static final int S_INIT_OK =
        defineStatus(CTRL_NONE, FAC_GENERIC, CODE_INIT_OK, SEV_SUCCESS,
                     "init ok");

    public static final int S_INIT_FAIL =
        defineStatus(CTRL_NONE, FAC_GENERIC, CODE_INIT_FAIL, SEV_ERROR,
                     "init failed");

    public static final int S_HW_NOT_PRESENT =
        defineStatus(CTRL_NONE, FAC_GENERIC, CODE_HW_NOT_PRESENT, SEV_ERROR,
                     "hardware not present");

    public static final int S_TEST_OK =
        defineStatus(CTRL_NONE, FAC_GENERIC, CODE_TEST_OK, SEV_SUCCESS,
                     "test ok");

    public static final int S_HW_FAULT =
        defineStatus(CTRL_NONE, FAC_GENERIC, CODE_HW_FAULT, SEV_ERROR,
                     "hardware fault");

    // Terminator framework codes (separate facility)
    public static final int TERM_TEST_TERMINATED_OK =
        defineStatus(CTRL_NONE, FAC_TERMINATOR, CODE_TERMINATED_OK, SEV_SUCCESS,
                     "test terminated by terminator (ok)");

    public static final int TERM_TEST_TERMINATED_BAD =
        defineStatus(CTRL_NONE, FAC_TERMINATOR, CODE_TERMINATED_BAD, SEV_ERROR,
                     "test terminated by fault");

    private DiagStatus32() {}
}
