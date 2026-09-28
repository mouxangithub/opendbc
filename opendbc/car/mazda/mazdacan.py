from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import Buttons

# Captured empty radar tracks required by the body ECU for stop-and-go. Only the counter
# nibble changes; 0x364 carries the advertised lead when present.
RADAR_STATIC_MSG = (0x499, bytes.fromhex("0008c00000000000"))
RADAR_TRACK_MSGS = {
  0x361: bytes.fromhex("fff7fefe1fc00080"),
  0x362: bytes.fromhex("fff7fefe1fc78c80"),
  0x363: bytes.fromhex("fff7fefe1fc00000"),
  0x364: bytes.fromhex("fff7fefe1fc00000"),
  0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
  0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
}
LEAD_TRACK_ADDR = 0x364
# Constant bytes for an occupied 0x364 track. create_lead_track replaces its measurements.
LEAD_TRACK_TEMPLATE = bytes.fromhex("000e00001c000000")
DIST_OBJ_SCALE = 0.0625   # m per bit, DIST_OBJ and RELV_OBJ share it
DIST_OBJ_MAX = 255.875    # m, the full-scale DIST_OBJ reading a track can carry

# The G46L radar (2016.5 bodies) sends only this static frame and no track messages at
# all, so the lead rides CRZ_CTRL alone; fully static — no counter, no checksum.
G46L_RADAR_STATIC_MSG = (0x499, bytes.fromhex("0098400000000000"))


def crz_info_checksum(dat: bytes) -> int:
  # Invert the sum of the first seven bytes, excluding STOPPING and RESUME_UNLATCHING.
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04) - (dat[6] & 0x40)) & 0xFF)) & 0xFF


def create_acc_command(packer, bus, counter, accel, *, long_active, acc_available,
                       brake_pressed=False, stopping=False, resume_unlatching=False):
  # CRZ_INFO replaces the disabled radar's acceleration command and armed-idle state.
  values = {
    "ERROR_STATUS": 1,
    "STATIC_1": 0x7ff,
    "CTR": counter % 16,
    "ACCEL_CMD": accel if long_active else 4.094,  # stock non-controlling sentinel
    "NEW_SIGNAL_7": int(long_active or acc_available),
  }
  if long_active:
    values.update({
      "ACC_ACTIVE": 1,
      "ACC_SET_ALLOWED": 1,
      "STOPPING": int(stopping),
      "STOPPING_2": int(stopping),
      "RESUME_UNLATCHING": int(resume_unlatching),
    })
  elif acc_available:
    values["ACC_SET_ALLOWED"] = int(not brake_pressed)

  dat = packer.make_can_msg("CRZ_INFO", bus, values)[1]
  values["CHKSUM"] = crz_info_checksum(dat)
  return packer.make_can_msg("CRZ_INFO", bus, values)


def create_crz_ctrl(packer, bus, long_active, acc_available, gap_setting, radar_has_lead, stop_go_phase, acc_active_2,
                    *, hbc_request=False):
  # CRZ_CTRL replaces radar cruise state and mirrors stop phase and driver gap selection.
  # NEW_SIGNAL_3 (bit 13) relays the camera's high-beam request (0x440 BIT2) as the stock radar
  # does; the body raises the lamps and the cluster's green HBC light on it.
  values = {
    "MSG_1_INV": 1,
    "MSG_1_INV_COPY": 1,
    "NEW_SIGNAL_8": 1,
    "CRZ_ACTIVE": int(long_active),
    "CRZ_AVAILABLE": int(long_active or acc_available),
    "DISTANCE_SETTING": gap_setting,
    "RADAR_HAS_LEAD": int(radar_has_lead),
    "RADAR_LEAD_RELATIVE_DISTANCE": stop_go_phase,
    "ACC_ACTIVE_2": int(acc_active_2),
    "NEW_SIGNAL_3": int(hbc_request),
  }
  return packer.make_can_msg("CRZ_CTRL", bus, values)


def create_lead_track(d_rel: float, v_rel: float) -> bytes:
  """Encode the advertised lead in the camera's track slot.

  Range must advance with relative velocity between measurements. RELV_OBJ uses positive
  values for an opening lead.
  """
  dist = round(min(max(d_rel, 0.), DIST_OBJ_MAX) / DIST_OBJ_SCALE)
  relv = round(min(max(v_rel, -64.), 63.9375) / DIST_OBJ_SCALE) & 0x7ff
  dat = bytearray(LEAD_TRACK_TEMPLATE)
  dat[0] = dist >> 4
  dat[1] = ((dist & 0xf) << 4) | (dat[1] & 0x0f)
  dat[3] = relv >> 3
  dat[4] = ((relv & 0x7) << 5) | (dat[4] & 0x1f)
  return bytes(dat)


def create_radar_frames(bus, counter, lead, g46l=False):
  """lead is the (dRel, vRel) of the object to advertise on 0x364, or None for an empty slot."""
  if g46l:
    return [CanData(G46L_RADAR_STATIC_MSG[0], G46L_RADAR_STATIC_MSG[1], bus)]
  frames = [CanData(RADAR_STATIC_MSG[0], RADAR_STATIC_MSG[1], bus)]
  for addr, dat in RADAR_TRACK_MSGS.items():
    if lead is not None and addr == LEAD_TRACK_ADDR:
      dat = create_lead_track(*lead)
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xf0) | (counter % 16)]), bus))
  return frames


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {
    "LKAS_REQUEST": apply_torque,
    "CTR": ctr,
    "ERR_BIT_1": er1,
    "LINE_NOT_VISIBLE": lnv,
    "LDW": ldw,
    "BIT_1": b1,
    "ERR_BIT_2": er2,
    "STEERING_ANGLE": steering_angle,
    "ANGLE_ENABLED": b2,
    "CHKSUM": csum
  }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  # Preserve camera LKAS state. Keep TJA modes clear because its state machine does not own
  # the injected steering command.
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


# The MADS white wheel: the dash draws it when the camera's own HUD frame carries TJA=2,
# and the body reads the same frame. The white bit is only ever XORed into an exact
# camera payload audited to be an idle frame, never a frame we composed. Every observed
# FSC idle family on TJA-declared cars is enumerated: the OFF family with its
# counter-nibble twins, the LINE_VISIBLE families and their high-beam variants, and the
# partial-lane LANE_LINES=3/4 encodings. Exact bases only; do not widen to a field-based
# rule until more captures are audited. No base may carry ERR_BIT, NO_ERR_BIT (byte 1 0x40,
# unsettled camera in carstate's takeover gate), LDW or a hands warning.
MADS_HUD_SAFE_BASE_PAYLOADS = frozenset(bytes.fromhex(h) for h in (
  "4201000000001040", "4201000000001060", "4221000000004040", "4221000000001040",
  "4221000000001060", "4201000000004040", "0221000000000040", "4201000000000040",
  "4221000000000040", "0221000000001040", "4102000000001040",
  "4122000000001040", "4102000000004040", "4122000000004040",
  "4221000000004060", "4122000000000040", "4103000000001040", "4104000000001040",
  "4123000000000040", "4124000000000040", "4123000000001040", "4124000000001040",
  "4123000000004040", "4124000000004040", "4102000000001060", "4102000000004060",
  "4122000000001060", "4122000000004060", "0122000000000040", "0122000000004040",
  "4202000000001040", "4102000000000040",
))
# OFF to WHITE is TJA 0 to 2 only: one bit, byte 4 0x20, XORed in, never a frame swap.
MADS_HUD_WHITE_TJA_XOR = bytes.fromhex("0000000020000000")
# 64-bit big-endian keep-mask over the bits an idle camera may still move between
# samples: TJA (byte 4, 0x70), TJA_TRANSITION (byte 3, 0x0C), and the unnamed byte-3
# transition bits 0x03. Do not clear byte-4 0x80 or unrelated byte-0 family bits.
# Every allowlisted base carries zero in the masked bits, so no two bases share a key.
CAM_LANEINFO_TJA_NORMALIZE_MASK = 0xFFFFFFF08FFFFFFF
_MADS_HUD_SAFE_BASE_BY_INT = {
  int.from_bytes(b, "big") & CAM_LANEINFO_TJA_NORMALIZE_MASK: b
  for b in MADS_HUD_SAFE_BASE_PAYLOADS
}


def white_hud_allowlist_base(fsc_raw: bytes | None) -> bytes | None:
  """The allowlisted idle base for the camera's current frame, TJA/transition bits ignored."""
  if fsc_raw is None or len(fsc_raw) != 8:
    return None
  return _MADS_HUD_SAFE_BASE_BY_INT.get(
    int.from_bytes(fsc_raw, "big") & CAM_LANEINFO_TJA_NORMALIZE_MASK
  )


def apply_mads_white_hud(fsc_raw: bytes | None, packed_dat: bytes, enabled: bool) -> bytes:
  """Set TJA=2 on the camera's own allowlisted idle frame, and on nothing else.

  packed_dat must be exactly the base the camera's current frame normalizes to: any other
  payload, or an unknown camera frame, passes through untouched.
  """
  if not enabled or len(packed_dat) != 8:
    return packed_dat
  if packed_dat != white_hud_allowlist_base(fsc_raw):
    return packed_dat
  return bytes(a ^ b for a, b in zip(packed_dat, MADS_HUD_WHITE_TJA_XOR, strict=True))


def is_mads_white_hud(dat: bytes) -> bool:
  """True when dat is an allowlisted base with only the WHITE TJA bit set."""
  if len(dat) != 8:
    return False
  base = bytes(a ^ b for a, b in zip(dat, MADS_HUD_WHITE_TJA_XOR, strict=True))
  return base in MADS_HUD_SAFE_BASE_PAYLOADS and dat != base


def create_button_cmd(packer, CP, counter, button):
  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  values = {
    # Never pressed by openpilot, on either bus. On the car's side it toggles MADS and arms
    # MRCC; on the camera's side it is the car's lane-keep switch (CAM_SETTINGS
    # LKAS_INERVENTION_ON1), and with that off the EPS applies no LKAS torque at all.
    "TJA_BUTTON": 0,

    "CAN_OFF": can,
    "CAN_OFF_INV": (can + 1) % 2,

    "SET_P": inc,
    "SET_P_INV": (inc + 1) % 2,

    "RES": res,
    "RES_INV": (res + 1) % 2,

    "SET_M": dec,
    "SET_M_INV": (dec + 1) % 2,

    "DISTANCE_LESS": 0,
    "DISTANCE_LESS_INV": 1,

    "DISTANCE_MORE": 0,
    "DISTANCE_MORE_INV": 1,

    "MODE_X": 0,
    "MODE_X_INV": 1,

    "MODE_Y": 0,
    "MODE_Y_INV": 1,

    "BIT1": 1,
    "BIT2": 1,
    "BIT3": 1,
    "CTR": (counter + 1) % 16,
  }

  return packer.make_can_msg("CRZ_BTNS", 0, values)


def create_mrcc_off_cmd(packer, counter):
  # The wheel's MRCC master press, active-low: every button bit 0 with its inversion 1,
  # the master signature in BIT1/BIT1_INV plus BIT2/BIT3, counter plus one. Only the
  # TJA-press cleanup sends it, and the panda pins the exact bytes in mazda_mrcc_off_msg_valid.
  values = {
    "CAN_OFF_INV": 1, "SET_P_INV": 1, "RES_INV": 1, "SET_M_INV": 1,
    "DISTANCE_LESS_INV": 1, "DISTANCE_MORE_INV": 1, "MODE_X_INV": 1, "MODE_Y_INV": 1,
    "BIT1_INV": 1, "BIT2": 1, "BIT3": 1,
    "CTR": (counter + 1) % 16,
  }
  return packer.make_can_msg("CRZ_BTNS", 0, values)
