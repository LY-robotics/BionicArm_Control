"""Public error codes returned by the motion-control API."""

OK = 0

ERR_SERIAL_OPEN_FAILED = 1001
ERR_NOT_CONNECTED = 1002

ERR_CAN_TX_FAILED = 1101
ERR_CAN_RX_TIMEOUT = 1102
ERR_PROTOCOL_RESPONSE = 1103

ERR_INVALID_JOINT_KEY = 1201
ERR_OUT_OF_LIMIT = 1202
ERR_INVALID_ARGUMENT = 1203

ERR_WAIT_REACH_TIMEOUT = 1301

ERR_IK_NO_SOLUTION = 1401
ERR_MOTION_BUSY = 1402
ERR_MOTION_CANCELLED = 1403

ERR_UNKNOWN = 1900

ERR_TEXT = {
    OK: "OK",
    ERR_SERIAL_OPEN_FAILED: "Serial port open failed",
    ERR_NOT_CONNECTED: "Arm is not connected",
    ERR_CAN_TX_FAILED: "CAN transmit failed",
    ERR_CAN_RX_TIMEOUT: "CAN receive timeout",
    ERR_PROTOCOL_RESPONSE: "Invalid or missing motor response",
    ERR_INVALID_JOINT_KEY: "Invalid joint key",
    ERR_OUT_OF_LIMIT: "Joint target out of limit",
    ERR_INVALID_ARGUMENT: "Invalid argument",
    ERR_WAIT_REACH_TIMEOUT: "Wait reach timeout",
    ERR_IK_NO_SOLUTION: "Inverse kinematics has no solution",
    ERR_MOTION_BUSY: "Motion is already running",
    ERR_MOTION_CANCELLED: "Motion was cancelled",
    ERR_UNKNOWN: "Unknown error",
}


def err_text(errcode: int) -> str:
    return ERR_TEXT.get(int(errcode), ERR_TEXT[ERR_UNKNOWN])
