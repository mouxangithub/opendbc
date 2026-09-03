import numpy as np
from opendbc.car import structs
from opendbc.car.byd.values import CanBus, CarControllerParams

GearShifter = structs.CarState.GearShifter
VisualAlert = structs.CarControl.HUDControl.VisualAlert


def byd_checksum(byte_key, dat):
    first_bytes_sum = sum(byte >> 4 for byte in dat)
    second_bytes_sum = sum(byte & 0xF for byte in dat)
    remainder = second_bytes_sum >> 4
    second_bytes_sum += byte_key >> 4
    first_bytes_sum += byte_key & 0xF
    first_part = ((-first_bytes_sum + 0x9) & 0xF)
    second_part = ((-second_bytes_sum + 0x9) & 0xF)
    return (((first_part + (-remainder + 5)) << 4) + second_part) & 0xFF


# MPC -> Panda -> EPS
def create_steering_control(packer, CP, cam_msg: dict, req_torque, req_prepare, active, hud_control, counter):
    values = {}
    values = {s: cam_msg[s] for s in [
        "AutoFullBeamState",
        "LeftLaneState",
        "LKAS_Config",
        "SETME2_0x1",
        "MPC_State",
        "AutoFullBeam_OnOff",
        "LKAS_Output",
        "LKAS_Active",
        "SETME3_0x0",
        "TrafficSignRecognition_OnOff",
        "SETME4_0x0",
        "SETME5_0x1",
        "RightLaneState",
        "LKAS_State",
        "TrafficSignRecognition_Result",
        "LKAS_AlarmType",
        "SETME7_0x3",
    ]}

    values["ReqHandsOnSteeringWheel"] = 0
    values["LKAS_ReqPrepare"] = req_prepare
    values["Counter"] = counter

    if active:
        mpc_state = values["MPC_State"] #2: Cancelling lkas control
        values.update({
            "LKAS_Output": req_torque,
            "LKAS_Active": 1,
            "LKAS_State": 4 if (mpc_state == 2) else 2,
            "LeftLaneState":  3 if hud_control.leftLaneDepart else int(hud_control.leftLaneVisible) + 1,
            "RightLaneState": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
        })
    else: # Note: This disables the stock AEB feature: turn steering wheel while close impacting obstacles in front.
        values.update({
            "LKAS_Output": 0,
            "LKAS_Active": 0,
        })

    data = packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_MPC_STATE", CanBus.ESC, values)


# op long control
def acc_cmd(packer, CP, cam_msg: dict, mrr_leaddist, accel, rfss, sss, longActive):
    values = {}

    values = {s: cam_msg[s] for s in [
        "AccelCmd",
        "ComfortBandUpper",
        "ComfortBandLower",
        "JerkUpperLimit",
        "SETME1_0x1",
        "JerkLowerLimit",
        "ResumeFromStandstill",
        "StandstillState",
        "BrakeBehaviour",
        "AccReqNotStandstill",
        "AccControlActive",
        "AccOverrideOrStandstill",
        "EspBehaviour",
        "Counter",
        "SETME2_0xF",
    ]}

    jerk_base_upper = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_upper_fp)
    jerk_base_lower = np.interp(mrr_leaddist, CarControllerParams.K_jerk_xp, CarControllerParams.K_jerk_base_lower_fp)

    if (accel < 0): #use lower factor
        jerk_upper = jerk_base_upper
        jerk_lower = jerk_base_lower + accel * CarControllerParams.K_accel_jerk_lower
    else:
        jerk_upper = jerk_base_upper + accel * CarControllerParams.K_accel_jerk_upper
        jerk_lower = jerk_base_lower

    if longActive:
        values.update({
            "AccelCmd": accel,
            "ComfortBandUpper": 0.05 if mrr_leaddist > 50 else 0.10,
            "ComfortBandLower": 0.05 if mrr_leaddist > 50 else 0.10,
            "JerkUpperLimit": jerk_upper,
            "JerkLowerLimit": jerk_lower,
            "ResumeFromStandstill": rfss,
            "StandstillState": sss,
        })

    data = packer.make_can_msg("ACC_CMD", CanBus.ESC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_CMD", CanBus.ESC, values)


# send fake torque feedback from eps to trick MPC, preventing DTC, so that safety features such as AEB still working
def create_fake_318(packer, CP, esc_msg: dict, faketorque, laks_reqprepare, laks_active, enabled, counter):
    values = {}

    values = {s: esc_msg[s] for s in [
        "LKAS_Prepared",
        "CruiseActivated",
        "TorqueFailed",
        "SETME1_0x1",
        "SteerWarning",
        "SteerErrorCode",
        "MainTorque",
        "SETME3_0x1",
        "SETME4_0x3",
        "SteerDriverTorque",
        "SETME5_0xFF",
        "SETME6_0xFFF",
    ]}

    values["ReportHandsNotOnSteeringWheel"] = 0
    values["Counter"] = counter

    if enabled:
        if laks_active:
            values.update({
                "LKAS_Prepared": 0,
                "CruiseActivated": 1,
                "MainTorque": faketorque,
            })
        elif laks_reqprepare:
            values.update({
                "LKAS_Prepared": 1,
                "CruiseActivated": 0,
                "MainTorque": 0,
            })
        else:
            values.update({
                "LKAS_Prepared": 0,
                "CruiseActivated": 0,
                "MainTorque": 0,
            })

    data = packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)[1]
    values["CheckSum"] = byd_checksum(0xAF, data)
    return packer.make_can_msg("ACC_EPS_STATE", CanBus.MPC, values)


# --- ATTO3 (byd_general.dbc) helpers ---

CHECKSUM_KEY = 0xAF

# Camera steering-frame template fields and default.
# UNKNOWN (bytes 0-1), SET_ME_X01 and SET_ME_XE of STEERING_MODULE_ADAS are constant for the
# whole of a camera steering episode. They must not be varied frame to frame.
ATTO3_STEER_TEMPLATE_FIELDS = ("UNKNOWN", "SET_ME_X01", "SET_ME_XE")
ATTO3_STEER_TEMPLATE_DEFAULT = {"UNKNOWN": 2773, "SET_ME_X01": 1, "SET_ME_XE": 0xB}

# Every LKAS_HUD_ADAS field except the STEER_ACTIVE bits and the counter/checksum. openpilot
# only owns whether LKAS is shown as active; lane-line state, traffic sign recognition, high
# beam assist and the PT2-PT5 / SET_ME_* fields belong to the camera and blank out unrelated
# driver-assist icons if we zero them, so they are mirrored from its copy read on bus 2.
ATTO3_LKAS_HUD_PASSTHROUGH = ("LSS_STATE", "SETTINGS", "SET_ME_XFF", "SET_ME_X5F", "SET_ME_1_2",
                              "TSR", "HMA", "HAND_ON_WHEEL_WARNING", "PT2", "PT3", "PT4", "PT5")


def atto3_create_steering_control(packer, apply_angle, template, idx):
    values = {
        "STEER_ANGLE": apply_angle,
        "STEER_REQ": 1,
        "STEER_REQ_ACTIVE_LOW": 0,
        "UNKNOWN": template["UNKNOWN"],
        "SET_ME_X01": template["SET_ME_X01"],
        "SET_ME_XE": template["SET_ME_XE"],
        "SET_ME_FF": 0xFF,
        "SET_ME_F": 0xF,
        "SET_ME_1_1": 1,
        "SET_ME_1_2": 1,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,
    }
    msg = packer.make_can_msg("STEERING_MODULE_ADAS", CanBus.ESC, values)
    values["CHECKSUM"] = byd_checksum(CHECKSUM_KEY, msg[1])
    return packer.make_can_msg("STEERING_MODULE_ADAS", CanBus.ESC, values)


def atto3_create_lkas_hud(packer, cam, idx):
    values = {
        "STEER_ACTIVE_1_1": 1,
        "STEER_ACTIVE_1_2": 1,
        "STEER_ACTIVE_1_3": 1,
        "STEER_ACTIVE_ACTIVE_LOW": 0,
        **{k: cam[k] for k in ATTO3_LKAS_HUD_PASSTHROUGH},
        "COUNTER": idx % 16,
        "CHECKSUM": 0,
    }
    msg = packer.make_can_msg("LKAS_HUD_ADAS", CanBus.ESC, values)
    values["CHECKSUM"] = byd_checksum(CHECKSUM_KEY, msg[1])
    return packer.make_can_msg("LKAS_HUD_ADAS", CanBus.ESC, values)


def atto3_create_acc_control(packer, accel, acc_enabled, idx):
    # ACCEL_CMD DBC scaling is (1, -100): physical = raw - 100. Pass the physical
    # value and let the packer apply the offset.
    accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    acc_on_1 = 1 if acc_enabled else 0
    acc_on_2 = 1 if acc_enabled else 0
    cmd_req_active_low = 0 if acc_enabled else 1
    acc_controllable_and_on = 1 if acc_enabled else 0
    acc_req_not_standstill = 1 if abs(accel) > 0 else 0

    values = {
        "ACCEL_CMD": accel,
        "ACC_ON_1": acc_on_1,
        "ACC_ON_2": acc_on_2,
        "CMD_REQ_ACTIVE_LOW": cmd_req_active_low,
        "ACC_CONTROLLABLE_AND_ON": acc_controllable_and_on,
        "ACC_REQ_NOT_STANDSTILL": acc_req_not_standstill,
        "SET_ME_25_1": 0x25,
        "SET_ME_25_2": 0x25,
        "SET_ME_XF": 0xF,
        "SET_ME_X8": 0x8,
        "SET_ME_1": 1,
        "ACCEL_FACTOR": 10,
        "DECEL_FACTOR": 10,
        "STANDSTILL_STATE": 0,
        "ACC_OVERRIDE_OR_STANDSTILL": 0,
        "STANDSTILL_RESUME": 0,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,
    }
    msg = packer.make_can_msg("ACC_CMD", CanBus.ESC, values)
    values["CHECKSUM"] = byd_checksum(CHECKSUM_KEY, msg[1])
    return packer.make_can_msg("ACC_CMD", CanBus.ESC, values)


def atto3_create_acc_hud(packer, acc_active, set_speed, lead_visible, idx):
    set_speed_dbc = int(set_speed * 2) if set_speed > 0 else 0
    set_speed_dbc = max(0, min(255, set_speed_dbc))

    values = {
        "ACC_ON1": 1 if acc_active else 0,
        "ACC_ON2": 1 if acc_active else 0,
        "SET_SPEED": set_speed_dbc,
        "SET_DISTANCE": 2,
        "SET_ME_XF": 0xF,
        "SET_ME_XFF": 0xFF,
        "COUNTER": idx % 16,
        "CHECKSUM": 0,
    }
    msg = packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)
    values["CHECKSUM"] = byd_checksum(CHECKSUM_KEY, msg[1])
    return packer.make_can_msg("ACC_HUD_ADAS", CanBus.ESC, values)
