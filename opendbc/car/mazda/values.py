from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

import numpy as np

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_MAX = 800                # theoretical max_steer 2047
  STEER_DELTA_UP = 10             # torque increase per refresh
  STEER_DELTA_DOWN = 25           # torque decrease per refresh
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 1     # weight driver torque
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    if CP.flags & MazdaFlags.GEN2:
      # GEN2 limits (e.g., MAZDA_3_2019); ported from source fork's MAZDA_3_2019 path
      self.STEER_MAX = 8000
      self.STEER_DELTA_UP = 45
      self.STEER_DELTA_DOWN = 80
      self.STEER_DRIVER_ALLOWANCE = 1400
      self.STEER_DRIVER_MULTIPLIER = 5
      self.STEER_DRIVER_FACTOR = 1
      self.STEER_ERROR_MAX = 3500

    # Torque-interceptor limits, used when MazdaFlags.TORQUE_INTERCEPTOR is set
    self.TI_STEER_MAX = 600
    self.TI_STEER_DELTA_UP = 6
    self.TI_STEER_DELTA_DOWN = 15
    self.TI_STEER_DRIVER_ALLOWANCE = 15
    self.TI_STEER_DRIVER_MULTIPLIER = 40
    self.TI_STEER_DRIVER_FACTOR = 1
    self.TI_STEER_ERROR_MAX = 350


def apply_ti_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):
  # Alternate steer-torque limits used when a torque interceptor is installed.
  # Ported from FrogPilot source fork: selfdrive/car/__init__.py:112-131.
  # Kept Mazda-internal so we don't pollute opendbc.car.lateral with brand-specific helpers.

  # limits due to driver torque
  driver_max_torque = LIMITS.TI_STEER_MAX + (LIMITS.TI_STEER_DRIVER_ALLOWANCE +
                                             driver_torque * LIMITS.TI_STEER_DRIVER_FACTOR) * LIMITS.TI_STEER_DRIVER_MULTIPLIER
  driver_min_torque = -LIMITS.TI_STEER_MAX + (-LIMITS.TI_STEER_DRIVER_ALLOWANCE + driver_torque *
                                              LIMITS.TI_STEER_DRIVER_FACTOR) * LIMITS.TI_STEER_DRIVER_MULTIPLIER
  max_steer_allowed = max(min(LIMITS.TI_STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.TI_STEER_MAX, driver_min_torque), 0)
  apply_torque = np.clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = np.clip(apply_torque, max(apply_torque_last - LIMITS.TI_STEER_DELTA_DOWN, -LIMITS.TI_STEER_DELTA_UP),
                           apply_torque_last + LIMITS.TI_STEER_DELTA_UP)
  else:
    apply_torque = np.clip(apply_torque, apply_torque_last - LIMITS.TI_STEER_DELTA_UP,
                           min(apply_torque_last + LIMITS.TI_STEER_DELTA_DOWN, LIMITS.TI_STEER_DELTA_UP))

  return int(round(float(apply_torque)))


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags. Bit values mirror panda's safety_mazda flags so the same int can be passed
  # through ret.safetyConfigs[0].safetyParam without translation.
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1
  # Gen 2 hardware (e.g., MAZDA_3_2019): different DBC, larger steer envelope, different camera protocol
  GEN2 = 2
  # Torque interceptor add-on hardware (third-party); requires apply_ti_steer_torque_limits and TI state machine
  TORQUE_INTERCEPTOR = 8
  # GEN2 openpilot longitudinal control. Set when alpha_long is opted in; gates panda safety
  # ACCEL_CMD validation on MAZDA_2019_ACC TX. Off-by-default to preserve stock-ACC passthrough.
  LONG = 16


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1

  def init(self):
    if self.flags & MazdaFlags.GEN2:
      self.dbc_dict = {Bus.pt: 'mazda_2019', Bus.cam: 'mazda_2019', Bus.body: 'mazda_2019'}


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MAZDA_CX5.specs,
  )
  MAZDA_3_2019 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2019-24")],
    MazdaCarSpecs(mass=3000 * CV.LB_TO_KG, wheelbase=2.725, steerRatio=18.8),
    flags=MazdaFlags.GEN2,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph
  TI_STEER_THRESHOLD = 6
  TI_DISABLE_SPEED = 0    # kph
  TI_ENABLE_SPEED = 0     # kph


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4
  TURN_ON = 5


class TI_STATE(IntEnum):
  # Torque-interceptor state machine values; ported verbatim from source fork
  # selfdrive/car/mazda/values.py (FrogPilot). The 4-value layout reflects the
  # actual TI firmware protocol used by the source codebase.
  DISCOVER = 0
  OFF = 1
  DRIVER_OVER = 2
  RUN = 3


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      whitelist_ecus=[Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.shiftByWire],
    ),
  ],
)

DBC = CAR.create_dbc_map()
