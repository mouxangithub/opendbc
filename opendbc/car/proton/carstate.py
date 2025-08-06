from cereal import car
from opendbc.can import CANDefine, CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.proton.values import DBC, CanBus, HUD_MULTIPLIER

class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
  # def __init__(self, CP):
  #   super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["TRANSMISSION"]['GEAR']
    self.set_distance_values = can_define.dv['PCM_BUTTONS']['SET_DISTANCE']
    self.is_cruise_latch = False
    self.acc_req = False
    self.hand_on_wheel_warning = False
    self.is_icc_on = False
    self.prev_angle = 0

    self.stock_lks_settings = 0
    self.stock_lks_settings2 = 0
    self.stock_ldp = 0
    self.stock_ldp_cmd = 0
    self.steer_dir = 0
    self.lkaDisabled = 0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
  # def update(self, cp):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.stock_lks_settings = cp.vl["ADAS_LKAS"]["STOCK_LKS_SETTINGS"]
    self.stock_ldp_cmd = cp.vl["ADAS_LKAS"]["STEER_CMD"]
    self.stock_lks_settings2 = cp.vl["ADAS_LKAS"]["SET_ME_1_1"]
    self.steer_dir = cp.vl["ADAS_LKAS"]["STEER_DIR"]
    self.stock_ldp = bool(cp.vl["LKAS"]["LANE_DEPARTURE_WARNING_RIGHT"]) or bool(cp.vl["LKAS"]["LANE_DEPARTURE_WARNING_LEFT"])
    # If cruise mode is ICC, make bukapilot control steering so it won't disengage.
    ret.lkaDisabled = not (bool(cp.vl["ADAS_LKAS"]["LKS_ENABLE"]) or self.is_icc_on)

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_B"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_B"],
    )
    ret.standstill = ret.vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["TRANSMISSION"]['GEAR'])

    ret.doorOpen = any([cp.vl["DOOR_LEFT_SIDE"]['BACK_LEFT_DOOR'],
                     cp.vl["DOOR_LEFT_SIDE"]['FRONT_LEFT_DOOR'],
                     cp.vl["DOOR_RIGHT_SIDE"]['BACK_RIGHT_DOOR'],
                     cp.vl["DOOR_RIGHT_SIDE"]['FRONT_RIGHT_DOOR']])

    ret.seatbeltUnlatched = cp.vl["SEATBELTS"]['RIGHT_SIDE_SEATBELT_ACTIVE_LOW'] == 1
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.brakeHoldActive = bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])

    disengage = ret.doorOpen or ret.seatbeltUnlatched or ret.brakeHoldActive
    if disengage:
      self.is_cruise_latch = False

    # gas pedal
    ret.gas = cp.vl["GAS_PEDAL"]['APPS_1']
    ret.gasPressed = ret.gas > 0.01

    # brake pedal
    ret.brake = cp.vl["BRAKE"]['BRAKE_PRESSURE']
    ret.brakePressed = bool(cp.vl["PARKING_BRAKE"]["BRAKE_PRESSED"])

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_MODULE"]['STEER_RATE'] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 5)
    self.hand_on_wheel_warning = bool(cp.vl["ADAS_LKAS"]["HAND_ON_WHEEL_WARNING"])
    self.is_icc_on = bool(cp.vl["PCM_BUTTONS"]["ICC_ON"])

    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    # Todo: get the real value
    ret.stockAeb = False
    ret.stockFcw = bool(cp.vl["FCW"]["STOCK_FCW_TRIGGERED"])

    self.acc_req = bool(cp.vl["ACC_CMD"]["ACC_REQ"])
    ret.cruiseState.available = any([cp.vl["PCM_BUTTONS"]["ACC_ON_OFF_BUTTON"], cp.vl["PCM_BUTTONS"]["GAS_OVERRIDE"]])

    #distance_val = int(cp.vl["PCM_BUTTONS"]['SET_DISTANCE'])
    # TODO: ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(distance_val, None))

    # engage and disengage logic
    if cp.vl["PCM_BUTTONS"]["ACC_SET"] == 0 and ret.brakePressed:
      self.is_cruise_latch = False

    if cp.vl["PCM_BUTTONS"]["ACC_SET"] != 0 and not ret.brakePressed:
      self.is_cruise_latch = True

    # set speed in range of 30 - 130kmh only
    self.cruise_speed = int(cp.vl["PCM_BUTTONS"]['ACC_SET_SPEED']) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = bool(cp.vl["ACC_CMD"]["STANDSTILL2"])
    ret.cruiseState.nonAdaptive = False

    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    if ret.brakePressed or (not self.acc_req and not ret.cruiseState.standstill):
      self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch

    # button presses
    ret.leftBlinker = bool(cp.vl["LEFT_STALK"]["LEFT_SIGNAL"])
    ret.rightBlinker = bool(cp.vl["LEFT_STALK"]["RIGHT_SIGNAL"])
    ret.genericToggle = bool(cp.vl["LEFT_STALK"]["GENERIC_TOGGLE"])

    ret.espDisabled = bool(cp.vl["PARKING_BRAKE"]["ESC_ON"]) != 1

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH"]) or bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH_WARNING"])
      ret.rightBlindspot = bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH"]) or bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH_WARNING"])

    return ret

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.main_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.cam_bus),
    }
