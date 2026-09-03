from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs
from opendbc.car.structs import CarParams
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CarControllerParams:
  # --- shared longitudinal params ---
  ACCEL_MAX = 2.0
  ACCEL_MIN = -3.5
  ACC_STEP = 2    # 50hz

  STEER_STEP = 2  # 100/2=50hz

  # --- torque-control params (HAN/TANG/SONG/QIN) ---
  STEER_MAX = 300
  # Align with safety/modes/byd.h: max_rate_up/down = 17 per 100 ms safety tick.
  # Control runs at 50 Hz (STEER_STEP=2), so 5 steps per 100 ms -> per-step limit
  # should be <= 17/5 = 3.4. Use 3 to stay inside the safety envelope.
  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 3

  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50

  STEER_SOFTSTART_STEP = 6 # 20ms(50Hz) * 300 / 6 = 1000ms. This means the clip ceiling will be increased to 300 in 1000ms

  K_DASHSPEED = 0.0719088 #convert pulse to kph

  USE_STEERING_SPEED_LIMITER = False

  # op long control
  K_accel_jerk_upper = 0.1
  K_accel_jerk_lower = 0.5
  K_jerk_xp = [4,   10,   20,   40,   80]  # meters
  K_jerk_base_lower_fp = [-2.3, -1.8, -1.4, -1.0, -0.4]
  K_jerk_base_upper_fp = [0.8,  0.7,  0.6,  0.3,  0.2]

  # --- angle-control params (ATTO3 / byd_general.dbc) ---
  # STEERING_MODULE_ADAS.STEER_ANGLE is an absolute steering wheel angle target
  # (DBC factor 0.1 deg). The EPS is a position servo, so the command must stay
  # anchored to the measured angle.
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # EPS faults past ~90 deg of commanded wheel angle.
    90.,  # deg
    # deg of change allowed per CAN message (50 Hz).
    ([0., 5., 25.], [2.5, 1.5, 0.4]),
    ([0., 5., 25.], [2.5, 1.5, 0.6]),
  )
  # Hard windup guard: the command may never sit further than this from measured angle.
  MAX_ANGLE_ERROR = 12.0  # deg
  # Driver override threshold on DRIVER_EPS_TORQUE (column sensor, raw 0-255).
  STEER_DRIVER_ALLOWANCE_ANGLE = 80

  def __init__(self, CP):
    pass


#FD to be added later
class BydSafetyFlags(IntFlag):
  HAN_TANG_DMEV = 0x1 #pre 2021 models with veoneer mpc/radar solution
  TANG_DMI = 0x2 #note tang dmi is not tang dm
  SONG_PLUS_DMI = 0x4 #note song pro is similar but not song dmi
  QIN_PLUS_DMI = 0x8
  YUAN_PLUS_DMI_ATTO3 = 0x10 #yuan plus is atto3
  ATTO3_GENERAL = 0x20  # ATTO3 export model using byd_general.dbc + angle control


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  #todo add docs and harness info


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "byd_han_dmev_2020"})
  #todo add dbc for other models


class CAR(Platforms):
  BYD_HAN_DM_20 = BydPlatformConfig(
    [BydCarDocs("BYD HAN DM 2020")],
    CarSpecs(mass=2080., wheelbase=2.920, steerRatio=16.8, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )
  BYD_HAN_EV_20 = BydPlatformConfig(
    [BydCarDocs("BYD HAN EV 2020")],
    CarSpecs(mass=2100., wheelbase=2.959, steerRatio=16.8, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  #The following parameters are likely be incorrect, developers please fill and fix them.

  BYD_TANG_DM = BydPlatformConfig(
    [BydCarDocs("BYD TANG DM 2021")],
    CarSpecs(mass=2250., wheelbase=2.820, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_TANG_DMI_21 = BydPlatformConfig(
    [BydCarDocs("BYD TANG DMI 2021")],
    CarSpecs(mass=2153., wheelbase=2.820, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PLUS_DMI_21 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PLUS DMI 2021")],
    CarSpecs(mass=1785., wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PLUS_DMI_22 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PLUS DMI 2022")],
    CarSpecs(mass=1785., wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PLUS_DMI_23 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PLUS DMI 2023")],
    CarSpecs(mass=1785., wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_SONG_PRO_DMI_22 = BydPlatformConfig(
    [BydCarDocs("BYD SONG PRO DMI 2022")],
    CarSpecs(mass=1670., wheelbase=2.712, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  BYD_QIN_PLUS_DMI_23 = BydPlatformConfig(
    [BydCarDocs("BYD QIN PLUS DMI 2023")],
    CarSpecs(mass=1580., wheelbase=2.718, steerRatio=15.0, centerToFrontRatio=0.44, tireStiffnessFactor=1.0),
  )

  # BYD Yuan Plus shares the export ATTO3 platform (e3.0, byd_general.dbc,
  # absolute steering-wheel-angle command). Keep a distinct platform string for
  # fingerprints/docs but share the ATTO3_GENERAL control logic.
  BYD_YUAN_PLUS_DMI_22 = BydPlatformConfig(
    [BydCarDocs("BYD YUAN PLUS DMI 2022")],
    CarSpecs(mass=1625., wheelbase=2.720, steerRatio=14.8, centerToFrontRatio=0.44, tireStiffnessFactor=0.7983),
    dbc_dict={Bus.pt: "byd_general", Bus.cam: "byd_general"},
  )

  # ATTO3 export model: this is the platform validated by qzwf's angle-control port.
  # It uses byd_general.dbc and an absolute steering-wheel-angle command.
  BYD_ATTO3 = BydPlatformConfig(
    [BydCarDocs("BYD ATTO3 2022-24", support_type=SupportType.COMMUNITY)],
    CarSpecs(mass=1750., wheelbase=2.720, steerRatio=14.8, centerToFrontRatio=0.44, tireStiffnessFactor=0.7983),
    dbc_dict={Bus.pt: "byd_general", Bus.cam: "byd_general"},
  )


class LKASConfig:
  DISABLE = 0
  ALARM = 1
  LKA = 2
  ALARM_AND_LKA = 3


class CanBus:
  ESC = 0
  MRR = 1
  MPC = 2


FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[\x00-\xff]+",
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=CanBus.ESC,
    ),
  ],
)

PLATFORM_HANTANG_DMEV = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}
PLATFORM_TANG_DMI = {CAR.BYD_TANG_DMI_21}
PLATFORM_SONG_PLUS_DMI = {CAR.BYD_SONG_PLUS_DMI_21, CAR.BYD_SONG_PLUS_DMI_22, CAR.BYD_SONG_PLUS_DMI_23, CAR.BYD_SONG_PRO_DMI_22}
PLATFORM_QIN_PLUS_DMI = {CAR.BYD_QIN_PLUS_DMI_23}
# Yuan Plus shares the ATTO3_GENERAL angle-control logic but keeps its own
# safety flag (YUAN_PLUS_DMI_ATTO3) for telemetry/fingerprinting.
PLATFORM_ATTO3_GENERAL = {CAR.BYD_ATTO3, CAR.BYD_YUAN_PLUS_DMI_22}

# power train canbus is located and accessible in in MPC connector
MPC_ACC_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

# power train canbus contains mrr radar info
PT_RADAR_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

# use torque lat control, otherwise use angle mode
TORQUE_LAT_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

# use experimental long mode
EXP_LONG_CAR = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM}

# Cars using the byd_general.dbc angle-control port
ANGLE_LAT_CAR = PLATFORM_ATTO3_GENERAL

DBC = CAR.create_dbc_map()
