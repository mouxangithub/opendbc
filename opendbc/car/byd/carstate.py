import copy
import numpy as np

from opendbc.can import CANDefine, CANParser

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import Bus, create_button_events, DT_CTRL, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CanBus, LKASConfig, CarControllerParams, PLATFORM_ATTO3_GENERAL
from opendbc.car.byd import bydcan
from opendbc.sunnypilot.car.byd.mads import MadsCarState
from opendbc.sunnypilot.car.byd.carstate_ext import CarStateExt

import os
BYD_RADAR = os.getenv("BYD_RADAR") is not None

ButtonType = structs.CarState.ButtonEvent.Type

ATTO3_STEER_TEMPLATE_STABLE_FRAMES = 5
ATTO3_CRUISE_DROP_FRAMES = 5


class CarState(CarStateBase, MadsCarState, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)

    self.is_atto3_general = CP.carFingerprint in PLATFORM_ATTO3_GENERAL

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    if not self.is_atto3_general:
      self.shifter_values = can_define.dv["DRIVE_STATE"]["Gear"]

    self.speed_kph = 0

    self.mpc_lkas_config = 0

    self.acc_hud_adas_counter = 0
    self.acc_mpc_state_counter = 0
    self.acc_cmd_counter = 0
    self.eps_state_counter = 0

    self.eps_warning = False

    self.acc_active_last = False
    self.low_speed_alert = False
    self.lkas_allowed_speed = False

    self.lkas_prepared = False  #318, EPS to OP
    self.acc_state = 0
    self.adas_set_dist = 0

    self.mpc_laks_output = 0
    self.mpc_laks_active = False
    self.mpc_laks_reqprepare = False

    self.cam_lkas = 0
    self.cam_acc = 0
    self.esc_eps = 0

    self.mrr_leading_dist = 0

    self.btn_acc_cancel = 0
    self.btn_acc_set_reset = 0
    self.btn_acc_dist_inc = 0
    self.btn_acc_dist_dec = 0

    self.steeringRateDegAbs = 0

    # ATTO3 state
    self.atto3_lkas_hud = dict.fromkeys(bydcan.ATTO3_LKAS_HUD_PASSTHROUGH, 0)
    self.atto3_steer_template = None
    self._atto3_template_candidate = None
    self._atto3_template_frames = 0
    self.atto3_acc_off_frames = 0
    self.atto3_cruise_enabled_last = False
    self.atto3_prev_angle = 0.0
    self.atto3_button_states = {
      ButtonType.leftBlinker: False,
      ButtonType.rightBlinker: False,
      ButtonType.accelCruise: False,
      ButtonType.decelCruise: False,
      ButtonType.lkas: False,
      ButtonType.gapAdjustCruise: False,
    }

  def _atto3_update_steer_template(self, cam_steer) -> None:
    if not cam_steer["STEER_REQ"]:
      self._atto3_template_candidate = None
      self._atto3_template_frames = 0
      return

    template = {k: int(cam_steer[k]) for k in bydcan.ATTO3_STEER_TEMPLATE_FIELDS}
    if not any(template.values()):
      return

    if template == self._atto3_template_candidate:
      self._atto3_template_frames += 1
    else:
      self._atto3_template_candidate = template
      self._atto3_template_frames = 1

    if self._atto3_template_frames >= ATTO3_STEER_TEMPLATE_STABLE_FRAMES:
      self.atto3_steer_template = template

  def _update_atto3(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    # --- Steering ---
    ret.steeringAngleOffsetDeg = 0.0
    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
    ret.steeringRateDeg = (ret.steeringAngleDeg - self.atto3_prev_angle) / DT_CTRL
    self.atto3_prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"]
    ret.steeringTorqueEps = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(ret.steeringTorque > 80, 5)

    # --- Pedals ---
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 1.0
    brake_pedal = cp.vl["PEDAL"]["BRAKE_PEDAL"]
    ret.brakePressed = brake_pedal > 0.03

    # --- Gear ---
    gear_map = {
      1: structs.CarState.GearShifter.park,
      2: structs.CarState.GearShifter.reverse,
      4: structs.CarState.GearShifter.drive,
    }
    ret.gearShifter = gear_map.get(int(cp.vl["DRIVE_STATE"]["GEAR"]),
                                   structs.CarState.GearShifter.unknown)

    # --- Wheel speeds ---
    _SPD_CORR = 40.0 / 53.0
    fl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"] * _SPD_CORR / 3.6
    fr = cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"] * _SPD_CORR / 3.6
    rl = cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"] * _SPD_CORR / 3.6
    # WHEELSPEED_BR (bits 48-63): byte 7 is a constant status byte (0x41),
    # NOT the high byte of the wheel speed. DBC wrongly declares it as 16-bit.
    # Derive RR from the other three wheels to stay consistent with vEgoRaw.
    rr = (fl + fr + rl) / 3.0
    ret.wheelSpeeds.fl = fl
    ret.wheelSpeeds.fr = fr
    ret.wheelSpeeds.rl = rl
    ret.wheelSpeeds.rr = rr
    ret.vEgoRaw = (fl + fr + rl) / 3.0
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.05

    # --- Cruise / ACC ---
    acc_main = bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON1"]) or bool(cp_cam.vl["ACC_HUD_ADAS"]["ACC_ON2"])
    acc_engaged = (bool(cp_cam.vl["ACC_CMD"]["ACC_ON_1"]) and bool(cp_cam.vl["ACC_CMD"]["ACC_ON_2"])
                   and not bool(cp_cam.vl["ACC_CMD"]["CMD_REQ_ACTIVE_LOW"]))

    if acc_engaged:
      self.atto3_acc_off_frames = 0
    else:
      self.atto3_acc_off_frames += 1
    if self.atto3_acc_off_frames == 0:
      self.atto3_cruise_enabled_last = True
    elif self.atto3_acc_off_frames >= ATTO3_CRUISE_DROP_FRAMES:
      self.atto3_cruise_enabled_last = False

    ret.cruiseState.enabled = self.atto3_cruise_enabled_last
    ret.cruiseState.available = acc_main
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] * CV.KPH_TO_MS
    ret.cruiseState.standstill = False

    ret.stockLkas = False
    self.atto3_lkas_hud = {k: cp_cam.vl["LKAS_HUD_ADAS"][k] for k in self.atto3_lkas_hud}
    self._atto3_update_steer_template(cp_cam.vl["STEERING_MODULE_ADAS"])

    # --- Safety ---
    ret.seatbeltUnlatched = not bool(cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"])
    ret.doorOpen = any([
      cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
      cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
    ])

    # --- Blinkers ---
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])

    # --- Button events ---
    button_map = {
      ButtonType.leftBlinker: ("STALKS", "LEFT_BLINKER"),
      ButtonType.rightBlinker: ("STALKS", "RIGHT_BLINKER"),
      ButtonType.accelCruise: ("PCM_BUTTONS", "RES_BTN"),
      ButtonType.decelCruise: ("PCM_BUTTONS", "SET_BTN"),
      ButtonType.lkas: ("PCM_BUTTONS", "LKAS_ON_BTN"),
      ButtonType.gapAdjustCruise: ("PCM_BUTTONS", "DEC_DISTANCE_BTN"),
    }
    button_events = []
    for event_type, (msg_name, sig_name) in button_map.items():
      val = bool(cp.vl[msg_name][sig_name])
      if event_type == ButtonType.gapAdjustCruise:
        # INC_DISTANCE_BTN also maps to gapAdjustCruise
        inc_val = bool(cp.vl["PCM_BUTTONS"]["INC_DISTANCE_BTN"])
        if val != self.atto3_button_states[event_type] or inc_val != self.atto3_button_states[event_type]:
          event = structs.CarState.ButtonEvent.new_message()
          event.type = event_type
          event.pressed = val or inc_val
          button_events.append(event)
          self.atto3_button_states[event_type] = val or inc_val
      else:
        if val != self.atto3_button_states[event_type]:
          event = structs.CarState.ButtonEvent.new_message()
          event.type = event_type
          event.pressed = val
          button_events.append(event)
          self.atto3_button_states[event_type] = val
    ret.buttonEvents = button_events

  def _update_legacy(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.lkas_prepared = cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"]

    self.mpc_lkas_config = int(cp_cam.vl["ACC_MPC_STATE"]["LKAS_Config"])
    lkas_config_isAccOn = (self.mpc_lkas_config != LKASConfig.DISABLE)
    lkas_isMainSwOn = bool(cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"])

    lkas_hud_AccOn1 = bool(cp_cam.vl["ACC_HUD_ADAS"]["AccOn1"])
    self.acc_state  = cp_cam.vl["ACC_HUD_ADAS"]["AccState"]
    self.adas_set_dist = cp_cam.vl["ACC_HUD_ADAS"]["SetDistance"]

    prev_btn_acc_cancel = self.btn_acc_cancel
    prev_btn_acc_set_reset = self.btn_acc_set_reset
    prev_btn_acc_dist_inc = self.btn_acc_dist_inc
    prev_btn_acc_dist_dec = self.btn_acc_dist_dec

    self.btn_acc_cancel = cp.vl["PCM_BUTTONS"]["BTN_AccCancel"]
    self.btn_acc_set_reset = cp.vl["PCM_BUTTONS"]["BTN_AccUpDown_Cmd"]
    self.btn_acc_dist_inc = cp.vl["PCM_BUTTONS"]["BTN_AccDistanceIncrease"]
    self.btn_acc_dist_dec = cp.vl["PCM_BUTTONS"]["BTN_AccDistanceDecrease"]

    # use dash speedo as speed reference
    speed_raw = int(cp.vl["CARSPEED"]["CarDisplaySpeed"])
    speed_raw_kph = speed_raw * CarControllerParams.K_DASHSPEED
    correct_factor = np.interp(speed_raw_kph, [30, 60, 90, 120], [1., 1., 1., 1.])
    self.speed_kph = speed_raw_kph * correct_factor

    ret.vEgoRaw = float(self.speed_kph * CV.KPH_TO_MS) # KPH to m/s
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.yawRate = cp.vl["YAW_RATE"]["YawRate"] - cp.vl["YAW_RATE"]["YawRateOffset"]

    ret.standstill = (speed_raw == 0)

    if self.CP.minSteerSpeed > 0:
      if self.speed_kph > 0.5:
        self.lkas_allowed_speed = True
      elif self.speed_kph < 0.1:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    can_gear = int(cp.vl["DRIVE_STATE"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["STALKS"]["HeadLight"])
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSD_RADAR"]["LEFT_APPROACH"])
      ret.rightBlindspot = bool(cp.vl["BSD_RADAR"]["RIGHT_APPROACH"])

    ret.leftBlinker = bool(cp.vl["STALKS"]["LeftIndicator"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RightIndicator"])

    ret.steeringAngleOffsetDeg = 0
    ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]

    self.steeringRateDegAbs = cp.vl["EPS"]["SteeringAngleRate"]
    ret.steeringRateDeg = self.steeringRateDegAbs

    ret.steeringTorque = cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"]
    ret.steeringTorqueEps = cp.vl["ACC_EPS_STATE"]["MainTorque"]
    self.eps_warning = bool(cp.vl["ACC_EPS_STATE"]["SteerWarning"]) #Todo: some firmware have SteerWarning field asserted.
    self.eps_state_counter = int(cp.vl["ACC_EPS_STATE"]["Counter"])

    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 59, 5)

    ret.parkingBrake = (cp.vl["EPB"]["EPB_ActiveFlag"] == 1)

    brake_pedal = int(cp.vl["PEDAL"]["BrakePedal"])
    ret.brakePressed = (brake_pedal != 0)

    ret.seatbeltUnlatched = (cp.vl["BCM"]["DriverSeatBeltFasten"] != 1)

    ret.doorOpen = any([cp.vl["BCM"]["FrontLeftDoor"], cp.vl["BCM"]["FrontRightDoor"],
                        cp.vl["BCM"]["RearLeftDoor"],  cp.vl["BCM"]["RearRightDoor"]])

    gas_pedal = int(cp.vl["PEDAL"]["AcceleratorPedal"])
    ret.gasPressed = (gas_pedal != 0)

    ret.cruiseState.available = lkas_isMainSwOn and lkas_config_isAccOn and lkas_hud_AccOn1
    ret.cruiseState.enabled = self.acc_state in (3, 5)
    ret.cruiseState.standstill = ret.standstill
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SetSpeed"] * CV.KPH_TO_MS

    #Todo: some firmware have these fields asserted.
    ret.steerFaultTemporary = bool((self.acc_state == 7) or self.eps_warning)

    self.acc_active_last = ret.cruiseState.enabled

    self.mpc_laks_output = cp_cam.vl["ACC_MPC_STATE"]["LKAS_Output"] #use to fool mpc
    self.mpc_laks_reqprepare = cp_cam.vl["ACC_MPC_STATE"]["LKAS_ReqPrepare"] != 0 #use to fool mpc
    self.mpc_laks_active = cp_cam.vl["ACC_MPC_STATE"]["LKAS_Active"] != 0 #use to fool mpc

    self.acc_hud_adas_counter = cp_cam.vl["ACC_HUD_ADAS"]["Counter"]
    self.acc_mpc_state_counter = cp_cam.vl["ACC_MPC_STATE"]["Counter"]
    self.acc_cmd_counter = cp_cam.vl["ACC_CMD"]["Counter"]

    self.cam_lkas = copy.copy(cp_cam.vl["ACC_MPC_STATE"])
    self.cam_acc = copy.copy(cp_cam.vl["ACC_CMD"])
    self.esc_eps = copy.copy(cp.vl["ACC_EPS_STATE"])

    if BYD_RADAR:
      mrr_id = int(cp_cam.vl["RADAR_MRR"]["TargetID"])

      if mrr_id == 2: #1:left, 2:front, 3:right
        if bool(cp_cam.vl["RADAR_MRR"]["IsValid"]):
          self.mrr_leading_dist = int(cp_cam.vl["RADAR_MRR"]["LongDist"])
        else:
          self.mrr_leading_dist = 199

    ret.steerFaultPermanent = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"]) #EPS give up all inputs until restart

    ret.buttonEvents = [
      *create_button_events(self.btn_acc_cancel, prev_btn_acc_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.btn_acc_set_reset, prev_btn_acc_set_reset, {1: ButtonType.decelCruise, 3: ButtonType.accelCruise}),
      *create_button_events(self.btn_acc_dist_inc, prev_btn_acc_dist_inc, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.btn_acc_dist_dec, prev_btn_acc_dist_dec, {1: ButtonType.gapAdjustCruise}),
    ]

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    if self.is_atto3_general:
      self._update_atto3(ret, ret_sp, can_parsers)
    else:
      self._update_legacy(ret, ret_sp, can_parsers)

    MadsCarState.update_mads(self, ret, can_parsers)
    CarStateExt.update(self, ret, ret_sp, can_parsers)

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    if CP.carFingerprint in PLATFORM_ATTO3_GENERAL:
      pt_messages = [
        ("STEER_MODULE_2", 100),
        ("STEERING_TORQUE", 100),
        ("PEDAL", 50),
        ("DRIVE_STATE", 50),
        ("WHEEL_SPEED", 50),
        ("METER_CLUSTER", 10),
        ("PCM_BUTTONS", 20),
        ("STALKS", 10),
      ]
      cam_messages = [
        ("ACC_HUD_ADAS", 50),
        ("ACC_CMD", 50),
        ("LKAS_HUD_ADAS", 50),
        ("STEERING_MODULE_ADAS", 50),
      ]
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.ESC),
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, CanBus.MPC),
      }

    pt_messages = [
      # sig_address, frequency
      ("EPS", 100),
      ("CARSPEED", 50),
      ("PEDAL", 50),
      ("EPB", 1),
      ("ACC_EPS_STATE", 50),
      ("DRIVE_STATE", 50),
      ("STALKS", 1),
      ("BCM", 1),
      ("PCM_BUTTONS", 20),
      ("YAW_RATE", 50),
    ]

    if CP.enableBsm:
      pt_messages.append(("BSD_RADAR", 20))

    cam_messages = [
      ("ACC_HUD_ADAS", 50),
      ("ACC_CMD", 50),
      ("ACC_MPC_STATE", 50),
    ]
    if BYD_RADAR:
      cam_messages.append(("RADAR_MRR", 60))

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.ESC),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus.MPC),
    }
