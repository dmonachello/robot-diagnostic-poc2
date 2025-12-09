package frc.robot.diag.vendor.ctre;

import frc.robot.diag.core.DiagStatus32;

/**
 * Talon (CTRE) specific diagnostic status codes.
 */
public final class TalonDiagStatus {

    public static final int FAC_TALON  = 2;
    private static final int CTRL_NONE = 0;

    private static final int CODE_OPEN_OK    = 1;
    private static final int CODE_OPEN_FATAL = 2;
    private static final int CODE_CMD_OK     = 3;
    private static final int CODE_CMD_ERROR  = 4;

    public static final int S_OPEN_OK = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_TALON,
            CODE_OPEN_OK,
            DiagStatus32.SEV_SUCCESS,
            "Talon open ok");

    public static final int S_OPEN_FATAL = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_TALON,
            CODE_OPEN_FATAL,
            DiagStatus32.SEV_FATAL,
            "Talon open fatal");

    public static final int S_CMD_OK = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_TALON,
            CODE_CMD_OK,
            DiagStatus32.SEV_SUCCESS,
            "Talon command ok");

    public static final int S_CMD_ERROR = DiagStatus32.defineStatus(
            CTRL_NONE,
            FAC_TALON,
            CODE_CMD_ERROR,
            DiagStatus32.SEV_ERROR,
            "Talon command error");

    private TalonDiagStatus() {}
}
