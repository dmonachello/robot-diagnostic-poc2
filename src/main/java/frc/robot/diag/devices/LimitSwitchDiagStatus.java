package frc.robot.diag.devices;

import frc.robot.diag.core.DiagStatus32;

/**
 * Limit-switch specific diagnostic status codes (as a first-class device).
 */
public final class LimitSwitchDiagStatus {

    public static final int FAC_LIMIT_SWITCH = 3;
    private static final int CTRL_NONE = 0;

    private static final int CODE_INIT_OK    = 1;
    private static final int CODE_INIT_FAIL  = 2;
    private static final int CODE_READ_OK    = 3;
    private static final int CODE_READ_FAULT = 4;
    private static final int CODE_INPUT_NULL = 50;

    public static final int S_INIT_OK = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_LIMIT_SWITCH,
            CODE_INIT_OK,
            DiagStatus32.SEV_SUCCESS,
            "Limit switch init ok");

    public static final int S_INIT_FAIL = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_LIMIT_SWITCH,
            CODE_INIT_FAIL,
            DiagStatus32.SEV_ERROR,
            "Limit switch init failed");

    public static final int S_READ_OK = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_LIMIT_SWITCH,
            CODE_READ_OK,
            DiagStatus32.SEV_SUCCESS,
            "Limit switch read ok");

    public static final int S_READ_FAULT = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_LIMIT_SWITCH,
            CODE_READ_FAULT,
            DiagStatus32.SEV_ERROR,
            "Limit switch read fault");

    public static final int S_INPUT_NULL =
    DiagStatus32.defineStatus(
            CTRL_NONE,
            DiagStatus32.FAC_TERMINATOR,
            CODE_INPUT_NULL,
            DiagStatus32.SEV_ERROR,
            "limit switch input not initialized");

    private LimitSwitchDiagStatus() {}
}
