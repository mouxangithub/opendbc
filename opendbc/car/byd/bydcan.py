#!/usr/bin/env python3
import numpy as np

from opendbc.car.byd.tuning import Tuning
from opendbc.car.byd.values import BydFlags, CanBus


def byd_checksum(byte_key, dat):
  high_sum = sum(byte >> 4 for byte in dat)
  low_sum = sum(byte & 0xF for byte in dat)
  low_overflow = low_sum >> 4
  low_sum += byte_key >> 4
  high_sum += byte_key & 0xF
  high_nibble = (-high_sum + 9) & 0xF
  low_nibble = (-low_sum + 9) & 0xF
  return (((high_nibble + (-low_overflow + 5)) << 4) + low_nibble) & 0xFF


def _copy_values(src, keys):
  return {key: src[key] for key in keys}


def _make_msg_with_checksum(packer, name, bus, values, byte_key=175):
  dat = packer.make_can_msg(name, bus, values)[1]
  values["sig_xbjyma"] = byd_checksum(byte_key, dat)
  return packer.make_can_msg(name, bus, values)


def create_steering_control(packer, CP, cam_msg: dict, req_torque, req_prepare, active,
                            hud_control, keep_lkas_passive, counter):
  values = _copy_values(cam_msg, (
    "sig_dedkvi", "sig_xnxyjw", "sig_mfykom", "sig_pahrba", "sig_oiobwq", "sig_uprtqw",
    "sig_kqgoqc", "sig_ausukb", "sig_zyaysi", "sig_vvkmqz", "sig_pzvsvx", "sig_qgavhs",
    "sig_dwbngy", "sig_vehado", "sig_ukhsab", "sig_uwnlau", "sig_lfdhgc",
  ))
  values["sig_aohukz"] = 0
  values["sig_hfhxif"] = req_prepare
  values["sig_fghgbq"] = counter

  if active:
    old_vehado = values["sig_oiobwq"]
    values.update({
      "sig_kqgoqc": req_torque,
      "sig_ausukb": 1,
      "sig_vehado": 0 if keep_lkas_passive else (1 if old_vehado == 2 else 2),
      "sig_xnxyjw": 3 if hud_control.leftLaneDepart else int(hud_control.leftLaneVisible) + 1,
      "sig_dwbngy": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
    })
  else:
    values.update({"sig_kqgoqc": 0, "sig_ausukb": 0})

  return _make_msg_with_checksum(packer, "CID_HGZKCQ", CanBus.ESC, values)


def create_steering_control_angle_mode(packer, CP, cam_msg: dict, active, req_prepare,
                                       hud_control, keep_lkas_passive, counter):
  values = _copy_values(cam_msg, (
    "sig_dedkvi", "sig_xnxyjw", "sig_mfykom", "sig_pahrba", "sig_oiobwq", "sig_uprtqw",
    "sig_kqgoqc", "sig_hfhxif", "sig_ausukb", "sig_zyaysi", "sig_vvkmqz", "sig_pzvsvx",
    "sig_qgavhs", "sig_dwbngy", "sig_vehado", "sig_ukhsab", "sig_uwnlau",
  ))
  values["sig_lfdhgc"] = 0
  values["sig_aohukz"] = 0
  values["sig_hfhxif"] = int(req_prepare)
  values["sig_ausukb"] = int(active)
  values["sig_fghgbq"] = counter

  if active or req_prepare:
    values.update({
      "sig_vehado": 0 if keep_lkas_passive else 3,
      "sig_xnxyjw": 3 if hud_control.leftLaneDepart else int(hud_control.leftLaneVisible) + 1,
      "sig_dwbngy": 3 if hud_control.rightLaneDepart else int(hud_control.rightLaneVisible) + 1,
    })
  if active:
    values["sig_hfhxif"] = 0

  return _make_msg_with_checksum(packer, "CID_HGZKCQ", CanBus.ESC, values)


def create_angle_control(packer, CP, cam_msg: dict, req_angle, max_torque, active, req_prepare, counter):
  values = _copy_values(cam_msg, (
    "sig_hfquge", "sig_tqntde", "sig_hfhxif", "sig_ausukb",
    "sig_uipsvp", "sig_yjvxpi", "sig_hcyhoo",
  ))
  values["sig_fghgbq"] = counter
  if active:
    values.update({
      "sig_kqgoqc": req_angle,
      "sig_ausukb": 1,
      "sig_hfhxif": 0,
      "sig_hfquge": max_torque,
      "sig_tqntde": -max_torque,
    })
  else:
    values.update({"sig_kqgoqc": 0, "sig_ausukb": 0, "sig_hfhxif": int(req_prepare)})
  return _make_msg_with_checksum(packer, "CID_TCDUBH", CanBus.ESC, values)


def acc_cmd_modified_stock_long(packer, CP, cam_msg: dict, adas_set_dist, mrr_leaddist, modlongActive, counter):
  values = _copy_values(cam_msg, (
    "sig_zspfbe", "sig_umvvum", "sig_ffvcas", "sig_vfmqcq", "sig_civxub", "sig_vuisck",
    "sig_qvxncf", "sig_doptnp", "sig_zrsmfm", "sig_yjgifm", "sig_gbaqki",
    "sig_xkavyk", "sig_tvemyu",
  ))
  values["sig_fghgbq"] = counter
  if modlongActive:
    accel = values["sig_zspfbe"]
    jerk_u = values["sig_vfmqcq"]
    jerk_l = values["sig_vuisck"]
    if accel > 0:
      if adas_set_dist == 3:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_3BAR)
      elif adas_set_dist == 2:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_2BAR)
      elif adas_set_dist == 1:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_1BAR)
      else:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_POS_4BAR)
    else:
      if adas_set_dist == 3:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_3BAR)
      elif adas_set_dist == 2:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_2BAR)
      elif adas_set_dist == 1:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_1BAR)
      else:
        scale = np.interp(mrr_leaddist, Tuning.K_ACCEL_BP, Tuning.K_ACCEL_NEG_4BAR)
    values.update({"sig_zspfbe": accel * scale, "sig_vfmqcq": jerk_u, "sig_vuisck": jerk_l})
  return _make_msg_with_checksum(packer, "CID_RJDCMR", CanBus.ESC, values)


def acc_cmd(packer, CP, cam_msg: dict, byd_jerk, accel, rfss, sss, longActive, accmode, counter):
  values = _copy_values(cam_msg, (
    "sig_zspfbe", "sig_umvvum", "sig_ffvcas", "sig_vfmqcq", "sig_civxub", "sig_vuisck",
    "sig_qvxncf", "sig_doptnp", "sig_zrsmfm", "sig_yjgifm", "sig_gbaqki",
    "sig_xkavyk", "sig_tvemyu",
  ))
  values["sig_fghgbq"] = counter
  old_accmode = values["sig_gbaqki"]
  if longActive:
    values.update({
      "sig_zspfbe": accel,
      "sig_umvvum": byd_jerk.cb_upper,
      "sig_ffvcas": byd_jerk.cb_lower,
      "sig_vfmqcq": byd_jerk.jerk_u,
      "sig_vuisck": byd_jerk.jerk_l,
    })
  if old_accmode != 5:
    if rfss < 2:
      values.update({"sig_qvxncf": rfss})
    if sss < 2:
      values.update({"sig_doptnp": sss})
    if accmode < 8:
      values.update({"sig_gbaqki": accmode})
  return _make_msg_with_checksum(packer, "CID_RJDCMR", CanBus.ESC, values)


def create_fake_318(packer, CP, esc_msg: dict, faketorque, laks_reqprepare, laks_active,
                    fake_driver_torque, enabled, counter):
  values = _copy_values(esc_msg, (
    "sig_rcgbok", "sig_kwshuz", "sig_civxub", "sig_ozgtym", "sig_odtaoc", "sig_itfrci",
    "sig_kviyqk", "sig_pcoiep", "sig_bhnetn", "sig_ckxuov", "sig_ktcyrh",
  ))
  values["sig_opaoqh"] = 0
  values["sig_fghgbq"] = counter
  if enabled:
    if laks_active:
      values.update({"sig_rcgbok": 2, "sig_itfrci": faketorque, "sig_bhnetn": fake_driver_torque})
    elif laks_reqprepare:
      values.update({"sig_rcgbok": 1, "sig_itfrci": 0, "sig_bhnetn": fake_driver_torque})
    else:
      values.update({"sig_rcgbok": 0, "sig_itfrci": 0})
  return _make_msg_with_checksum(packer, "CID_IQMESB", CanBus.MPC, values)


def create_fake_1FC(packer, CP, esc_msg: dict, faketorque, laks_reqprepare, laks_active, enabled, counter):
  values = _copy_values(esc_msg, (
    "sig_rcgbok", "sig_kwshuz", "sig_civxub", "sig_bhnetn", "sig_eszmdp",
    "sig_itfrci", "sig_bymskd", "sig_qfglfx", "sig_jgmunz",
  ))
  values["sig_opaoqh"] = 0
  values["sig_fghgbq"] = counter
  if enabled:
    driver_override = values["sig_rcgbok"] == 3
    if CP.flags & BydFlags.ANGLE_DRIVER_OVERRIDE_RELEASE:
      driver_override = driver_override or values["sig_jgmunz"] >= 3 or values["sig_kwshuz"]
    if driver_override:
      values.update({"sig_rcgbok": 3, "sig_itfrci": 0})
    elif laks_active:
      values.update({"sig_rcgbok": 2, "sig_itfrci": faketorque})
    elif laks_reqprepare:
      values.update({"sig_rcgbok": 1, "sig_itfrci": 0})
    else:
      values.update({"sig_rcgbok": 0, "sig_itfrci": 0})
  return _make_msg_with_checksum(packer, "CID_KERMHK", CanBus.MPC, values)


def create_adas_hud(packer, cam_msg: dict, setSpeed, useCustomSpeed, counter):
  values = _copy_values(cam_msg, (
    "sig_gqnxpm", "sig_ldwxym", "sig_ijrdxw", "sig_ifqqtp", "sig_rqvixu", "sig_znmccg",
    "sig_civxub", "sig_hprcqm", "sig_ueqstp", "sig_ynhjzo", "sig_pahrba", "sig_tvpmqh",
    "sig_uppwhx", "sig_iprocm", "sig_ohsrtt", "sig_hgcmjs",
  ))
  values["sig_fghgbq"] = counter
  if useCustomSpeed:
    values.update({"sig_gqnxpm": setSpeed})
  return _make_msg_with_checksum(packer, "CID_XQYNBW", CanBus.ESC, values)


def create_mpc_pcm_button(packer, pt_msg: dict, freeze_updown_input, counter, updown_cmd):
  values = _copy_values(pt_msg, (
    "sig_tekixn", "sig_dzciox", "sig_ymtlod", "sig_lftkfn", "sig_ssvpvb", "sig_fehruy",
    "sig_xmstyv", "sig_uikswg", "sig_ebundx", "sig_tpueds", "sig_dzuevb", "sig_aleizt",
    "sig_fmnatb", "sig_jnpios", "sig_xaplnq",
  ))
  values["sig_fghgbq"] = counter
  values["sig_lggzjg"] = 1
  if freeze_updown_input:
    values.update({"sig_ymtlod": 0})
  elif updown_cmd:
    values.update({"sig_ymtlod": updown_cmd})
  return _make_msg_with_checksum(packer, "CID_YHMGPU", CanBus.MPC, values)


# --- ATTO3 (byd_general.dbc, kept from the local validated port) ---

CHECKSUM_KEY = 0xAF

ATTO3_STEER_TEMPLATE_FIELDS = ("UNKNOWN", "SET_ME_X01", "SET_ME_XE")
ATTO3_STEER_TEMPLATE_DEFAULT = {"UNKNOWN": 2773, "SET_ME_X01": 1, "SET_ME_XE": 0xB}

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
  accel = float(np.clip(accel, -3.5, 2.0))
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
