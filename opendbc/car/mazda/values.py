from dataclasses import dataclass, field
from enum import IntFlag, StrEnum

from opendbc.car import Bus, CarSpecs, DbcDict, DT_CTRL, PlatformConfig, Platforms
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.vin import Vin, is_valid_vin

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_FACTOR = 1         # from dbc
  # Keep steering deltas synchronized with this 100 Hz control rate.
  STEER_STEP = 1

  # The measured envelope's full scale, equal to the panda's max_torque for it.
  EPS_STEER_MAX = 1200  # theoretical max_steer 2047
  # Upstream's STEER_MAX: the scale params.toml's Mazda tunes, sunnypilot's NNLC models and the
  # manual torque override are expressed on. Every steering Mazda runs the envelope, so one
  # ratio converts them (latAccelFactor x TUNE_SCALE, friction / TUNE_SCALE).
  TUNE_STEER_MAX = 800
  TUNE_SCALE = EPS_STEER_MAX / TUNE_STEER_MAX

  ACCEL_MAX = 2.0   # m/s2
  ACCEL_MIN = -3.5  # m/s2

  # Longitudinal message periods in 100 Hz control frames.
  LONG_STEP = 2        # CRZ_INFO/CRZ_CTRL at 50 Hz, matching stock
  RADAR_STEP = 10      # radar static + track frames at 10 Hz
  RADAR_UDS_STEP = 50  # radar UDS traffic at 2 Hz: session control or tester present

  # Wait for the camera's cold-boot radar check before silencing the radar.
  FSC_SETTLE_T = 7.0           # observed-settled time before the teardown may start (check passed from 5.8 s)
  # This alive window detects a normal CRZ_INFO gap but does not establish ownership.
  STOCK_RADAR_ALIVE_T = 0.05
  # Complete this ownership guard after panda's matching radar-silence guard.
  PANDA_RADAR_SILENT_T = 1.0            # mazda.h MAZDA_RADAR_SILENT_FRAMES / 50 Hz PEDALS
  STOCK_RADAR_GUARD_MARGIN_T = 0.2
  STOCK_RADAR_GUARD_T = STOCK_RADAR_ALIVE_T + LONG_STEP * DT_CTRL + PANDA_RADAR_SILENT_T + STOCK_RADAR_GUARD_MARGIN_T  # 1.27 s
  RADAR_SESSION_LIMIT_T = 10.0  # per-attempt UDS budget
  # CAM_LANEINFO runs near 2 Hz, so its freshness window must exceed one period.
  CAM_LANEINFO_PERIOD_T = 0.563
  CAM_LANEINFO_FRESH_T = 1.5

  # The car's lane keep back on, the EPS re-arms: LKAS_BLOCK with TRACK_STATE for 3.00 to 3.08 s
  # from the edge whatever it is sent, then torque 0.02 to 0.38 s later (7 edges: routes
  # 0000024d, 00000105, and the camera's ERR recovery on 00000043).
  LKAS_REARM_T = 3.0           # no lift of the block counts before this
  LKAS_REARM_FAULT_T = 4.0     # the block it raises is not a fault for this long

  # Stock body-latched releases use a nine-frame RESUME_UNLATCHING pulse.
  RESUME_UNLATCH_LATCHED_T = 0.18  # s, 9 wire frames, the latched-family mode
  # Retry one unanswered body-latched release, then return control to the plan.
  RESUME_REPULSE_T = 1.0  # s after a latched release, GEAR.BRAKE_HOLD still set

  MAIN_OFF_DEBOUNCE_T = 0.1   # both PEDALS cruise bits low this long is a main-off; no transient dropout in 4026 segments

  # Debounce movement requests before releasing a standstill hold.
  RELEASE_DEBOUNCE_T = 0.2

  # Debounce lead visibility before advertising a radar track.
  LEAD_DEBOUNCE_T = 0.5

  # Relax the command after the body ECU takes ownership of the brake hold.
  ACCEL_HOLD_LATCHED = -0.001  # m/s2

  # Match stock's ACCEL_CMD ceiling during a latched release pulse.
  ACCEL_RESUME_PULSE_MAX = 0.25  # m/s2, latched releases only

  # Match stock's one-frame relaxation and subsequent release ramp.
  ACCEL_RELEASE_BAND = -0.26  # m/s2, the one-frame relax target at a never-latched release
  ACCEL_RELEASE_RAMP = 1.25   # m/s3, stock's release ramp (+25 raw per 50 Hz frame)

  # Permit a bounded breakaway ramp because Mazda longitudinal control has no integrator.
  ACCEL_BREAKAWAY_MAX = 1.45  # m/s2, ceiling for the still-stopped release ramp
  ACCEL_BREAKAWAY_T = 3.0  # s
  ACCEL_BREAKAWAY_OVERSHOOT = 0.75  # m/s2 above the plan the still-stopped ramp may climb

  # Shape positive commands like stock MRCC (tools/mazda_long/accel_profile.py, 158 stock routes).
  # Stock never asks for more than these by speed (p99 of the accelerating command, no lead).
  ACCEL_CEILING_BP = [0., 4., 9., 14., 18., 25.]  # m/s
  ACCEL_CEILING_V = [1.5, 1.75, 1.45, 1.05, 0.85, 0.65]  # m/s2
  # Stock builds positive accel at +12 raw per 50 Hz frame (0.6 m/s3) once rolling, 99.3% of
  # rising frames, and at its 1.25 m/s3 release ramp while pulling away. The plan steps faster
  # than both, which is the driver-felt harshness. Rolling builds a third quicker than stock on
  # purpose: the plan sees a lead pull away before the stock radar walk would. Applies above
  # zero only: brake release keeps the looser windup below so braking is never held longer
  # than the plan asks.
  ACCEL_BUILD_BP = [3., 6.]   # m/s
  ACCEL_BUILD_V = [1.25, 0.8]  # m/s3
  # Stock lifts the throttle at no more than 40 raw per 50 Hz frame (2.0 m/s3) in 99.98% of
  # falling positive frames. Applies to throttle modulation only (plan still >= 0); a plan
  # asking for brake falls through to the winddown limit so braking is never delayed.
  ACCEL_LIFT_LIMIT = -2.0  # m/s3
  # Limit upward plan-command slew in the brake region without delaying braking response.
  ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL     # m/s2 per frame
  ACCEL_WINDDOWN_LIMIT = -10.0 * DT_CTRL  # m/s2 per frame, clips only the p99.9+ steps

  def __init__(self, CP):
    # Every gen1 Mazda EPS runs the measured envelope; the interface sets one of the two bits.
    if CP.flags & MazdaFlags.EPS_HW:
      # Match the EPS hardware slew and panda safety limits in both directions.
      self.STEER_DELTA_UP = 12
      self.STEER_DELTA_DOWN = 12
      self.STEER_DRIVER_MULTIPLIER = 15   # tuned for the CX-5 EPS response
      # Use a sample window and margin to stay inside panda's fresher driver-torque envelope.
      self.STEER_DRIVER_SAMPLES = 10
      self.STEER_DRIVER_MARGIN = 2

      # A panda rejection resets its rate-limit reference to zero, so every later frame more
      # than one step from zero is rejected too and the EPS stops receiving 0x243. About 0.6 s
      # into that silence the EPS raises STEER_RATE.LKAS_FAULT and the camera faults 5.3 s
      # later; neither clears before the next ignition cycle. The EPS echoes the last request
      # it received in STEER_RATE.LKAS_REQUEST, so an echo that matches none of the recent
      # commands means they are being rejected, and the ramp restarts from zero, which the
      # panda accepts. See docs/zoompilot/mazda-lateral.md, "LKAS_FAULT".
      self.STEER_ECHO_HISTORY = 4            # commands the 83 Hz echo may lag behind
      self.STEER_ECHO_MISMATCH_FRAMES = 5    # 50 ms without a matching echo restarts the ramp

      # STEER_MAX scales normalized torque into counts; EPS_CEILING_LOOKUP is the applied limit.
      # Legacy firmware never commands below its 45 kph floor, so the low-speed scale is moot
      # there and the rest of the schedule is the same hardware.
      self.STEER_MAX = 1200        # theoretical max_steer 2047
      self.STEER_MAX_LOOKUP = ([0., 14.2, 14.5], [1200, 1200, 800])
      # Clamp to the measured applied-torque ceiling so controlsd can detect saturation.
      self.EPS_CEILING_LOOKUP = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5],
                                 [1148, 1132, 1092, 1048, 1012,  920,  808,  676,  620])

      if CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
        # Stop commanding after sustained zero delivery to avoid a camera steering fault. Use
        # LKAS_EFFECTIVE because LKAS_BLOCK may still permit partial delivery.
        self.STEER_UNDELIVERED_MIN = 200      # counts; below this the EPS rounds to zero anyway
        self.STEER_UNDELIVERED_FRAMES = 20    # 200 ms at 100 Hz

        # Alert only after sustained non-delivery above maneuvering speed. Suppress normal
        # low-speed standby blocks identified by LKAS_TRACK_STATE.
        self.STEER_UNDELIVERED_ALERT_FRAMES = 80    # 0.8 s at 100 Hz, on top of the latch's 0.2
        self.STEER_UNDELIVERED_ALERT_MIN_SPEED = 12. * CV.MPH_TO_MS
        # A block that began below this speed is the EPS's standby from a stop, whatever
        # LKAS_TRACK_STATE says later in it; only a block that began rolling can be a dropout.
        self.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED = 1.0  # m/s
    else:
      # Upstream's envelope. The interface no longer selects it for any Mazda; the panda keeps
      # it as the no-param default, so flags == 0 must still build.
      self.STEER_MAX = 800         # theoretical max_steer 2047
      self.STEER_DELTA_UP = 10
      self.STEER_DELTA_DOWN = 25
      self.STEER_DRIVER_MULTIPLIER = 1    # upstream stock


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


@dataclass(frozen=True, kw_only=True)
class MazdaCX5_2022CarSpecs(CarSpecs):
  tireStiffnessFactor: float = 1.0


class MazdaFlags(IntFlag):
  # GEN1 platforms share CAN messages and camera hardware.
  GEN1 = 1

  # EPS firmware that steers to zero: no speed floor, the 1200-count low-speed scale, and
  # LKAS_TRACK_STATE standby semantics.
  STEER_TO_ZERO_EPS = 2
  # Every other gen1 Mazda EPS: the same hardware on firmware that keeps the 45 kph floor. Same
  # envelope and tune; the floor, the non-delivery latch and alpha long stay with steer-to-zero.
  LEGACY_FW_EPS = 4
  # Everything keyed on the measured hardware rather than on what the firmware permits.
  EPS_HW = STEER_TO_ZERO_EPS | LEGACY_FW_EPS


class MazdaSafetyFlags(IntFlag):
  LONG = 1
  # Selects the steer-to-zero EPS envelope in panda safety.
  STEER_TO_ZERO_EPS = 2
  # Selects the same envelope for legacy firmware; a distinct bit so logs show the firmware.
  LEGACY_FW_EPS = 4


class WMI(StrEnum):
  JAPAN_PASSENGER = "JM1"   # Japan-built passenger cars
  JAPAN_CROSSOVER = "JM3"   # Japan-built crossovers
  MEXICO_PASSENGER = "3MZ"  # Mazda de Mexico (Mazda 3)
  # Export VINs without a model-year field use the EPS-swap fallback.
  OCEANIA_EXPORT = "JM0"


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017', Bus.radar: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1
  wmis: set[WMI] = field(default_factory=set)
  chassis_codes: set[str] = field(default_factory=set)
  years: set[str] = field(default_factory=set)


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5),
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'KF'}, years={'H', 'J', 'K', 'L', 'M'},  # 2017-21
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6),
    # This radar does not publish 0x361-0x366 tracks on bus 0.
    dbc_dict={Bus.pt: 'mazda_2017'},
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'TC'}, years={'G', 'H', 'J', 'K', 'L'},  # 2016-20
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0),
    wmis={WMI.JAPAN_PASSENGER, WMI.MEXICO_PASSENGER}, chassis_codes={'BN'}, years={'H', 'J'},  # 2017-18
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5),
    wmis={WMI.JAPAN_PASSENGER}, chassis_codes={'GL'}, years={'H', 'J', 'K', 'L', 'M'},  # 2017-21
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MazdaCarSpecs(mass=4409 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6),
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'TC'}, years={'M', 'N', 'P'},  # 2021-23
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MazdaCX5_2022CarSpecs(mass=3728 * CV.LB_TO_KG, wheelbase=2.698, steerRatio=18.1),  # 15.5 is factory spec; 18.1 from paramsd learner (2.9M samples)
    wmis={WMI.JAPAN_CROSSOVER}, chassis_codes={'KF'}, years={'N', 'P', 'R', 'S'},  # 2022-25
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


# Keep steer-to-zero firmware synchronized with the CX-5 2022 EPS entries in fingerprints.py.
STEER_TO_ZERO_EPS_FW = {
  b'KBST-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
}


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # After firmware matching fails, require VIN fields to identify one chassis platform.
  if not is_valid_vin(vin):
    return set()

  vin_obj = Vin(vin)
  chassis_code = vin_obj.vds[0:2]
  year = vin_obj.vis[0]

  candidates = set()
  for platform in CAR:
    platform_config = platform.config
    if vin_obj.wmi in platform_config.wmis and chassis_code in platform_config.chassis_codes and year in platform_config.years:
      candidates.add(platform)

  if len(candidates) == 1:
    carlog.error(f"Fingerprinted {next(iter(candidates))} by VIN")
    return {str(c) for c in candidates}

  # Only export VINs without model-year data continue to the EPS-swap fallback.
  if vin_obj.wmi != WMI.OCEANIA_EXPORT:
    return set()

  # Export-car swaps require a recognized EPS and an engine that identifies one platform.
  eps_fw = live_fw_versions.get((0x730, None), set())
  if not eps_fw & STEER_TO_ZERO_EPS_FW:
    return set()

  engine_fw = live_fw_versions.get((0x7e0, None), set())
  candidates = {platform for platform, ecus in offline_fw_versions.items()
                if engine_fw & set(ecus.get((Ecu.engine, 0x7e0, None), []))}
  if len(candidates) != 1:
    return set()

  carlog.error(f"Fingerprinted {next(iter(candidates))} by engine firmware behind a steer-to-zero EPS swap")
  return {str(c) for c in candidates}

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9-]{11,16}\x00{8,13}",
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
