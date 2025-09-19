#!/usr/bin/env python3
from math import fabs, exp
import numpy as np

from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, LateralAccelFromTorqueCallbackType
from opendbc.car.byd.values import CAR, CarControllerParams, CanBus, BydSafetyFlags, MPC_ACC_CAR, TORQUE_LAT_CAR, EXP_LONG_CAR, \
                                PLATFORM_HANTANG_DMEV, PLATFORM_TANG_DMI, PLATFORM_SONG_PLUS_DMI, PLATFORM_QIN_PLUS_DMI, PLATFORM_YUAN_PLUS_DMI_ATTO3
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType
NetworkLocation = structs.CarParams.NetworkLocation

NON_LINEAR_TORQUE_PARAMS = {
  CAR.BYD_HAN_DM_20: [1.807, 1.674, 0.04],
  CAR.BYD_HAN_EV_20: [1.807, 1.674, 0.04]
}

import os
BYD_RADAR = os.getenv("BYD_RADAR") is not None

class CarInterface(CarInterfaceBase):
    CarState = CarState
    CarController = CarController
    RadarInterface = RadarInterface
    @staticmethod
    def get_pid_accel_limits(CP, current_speed, cruise_speed):
      return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

    # Determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
    @staticmethod
    def get_steer_feedforward_volt(desired_angle, v_ego):
      desired_angle *= 0.02904609
      sigmoid = desired_angle / (1 + fabs(desired_angle))
      return 0.10006696 * sigmoid * (v_ego + 3.12485927)

    def get_steer_feedforward_function(self):
      return CarInterfaceBase.get_steer_feedforward_default

    def get_lataccel_torque_siglin(self) -> float:

        def torque_from_lateral_accel_siglin_func(lateral_acceleration: float) -> float:
          # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
          # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
          # This has big effect on the stability about 0 (noise when going straight)
          non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
          assert non_linear_torque_params, "The params are not defined"
          a, b, c, _ = non_linear_torque_params
          sig_input = a * lateral_acceleration
          sig = np.sign(sig_input) * (1 / (1 + exp(-fabs(sig_input))) - 0.5)
          steer_torque = (sig * b) + (lateral_acceleration * c)
          return float(steer_torque)

        lataccel_values = np.arange(-5.0, 5.0, 0.01)
        torque_values = [torque_from_lateral_accel_siglin_func(x) for x in lataccel_values]
        assert min(torque_values) < -1 and max(torque_values) > 1, "The torque values should cover the range [-1, 1]"
        return torque_values, lataccel_values

    def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
        if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
            torque_values, lataccel_values = self.get_lataccel_torque_siglin()
            def torque_from_lateral_accel_siglin(lateral_acceleration: float, torque_params: structs.CarParams.LateralTorqueTuning):
                return np.interp(lateral_acceleration, lataccel_values, torque_values)
            return torque_from_lateral_accel_siglin
        else:
            return self.torque_from_lateral_accel_linear

    def lateral_accel_from_torque(self) -> LateralAccelFromTorqueCallbackType:
      if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
        torque_values, lataccel_values = self.get_lataccel_torque_siglin()

        def lateral_accel_from_torque_siglin(torque: float, torque_params: structs.CarParams.LateralTorqueTuning):
          return np.interp(torque, torque_values, lataccel_values)
        return lateral_accel_from_torque_siglin
      else:
        return self.lateral_accel_from_torque_linear

    @staticmethod
    def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, is_release, docs) -> structs.CarParams: # type: ignore
        ret.brand = "byd"
        ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]

        ret.dashcamOnly = False
        #disable simple pt radar due to mpc solver issue in official OP. It works with carrot/sunny/forg.
        if BYD_RADAR:
            ret.radarUnavailable = False
        else:
            ret.radarUnavailable = True #candidate not in PT_RADAR_CAR


        ret.minEnableSpeed = -1.
        ret.enableBsm = 0x418 in fingerprint[CanBus.ESC]
        ret.transmissionType = TransmissionType.direct

        ret.minEnableSpeed = -1.
        ret.minSteerSpeed = 0.1 * CV.KPH_TO_MS

        ret.steerActuatorDelay = 0.05
        ret.steerLimitTimer = 0.4

        if candidate in PLATFORM_HANTANG_DMEV:
            ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.HAN_TANG_DMEV.value
        elif candidate in PLATFORM_TANG_DMI:
            ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.TANG_DMI.value
        elif candidate in PLATFORM_SONG_PLUS_DMI:
                    ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.SONG_PLUS_DMI.value
        elif candidate in PLATFORM_QIN_PLUS_DMI:
                    ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.QIN_PLUS_DMI.value
        elif candidate in PLATFORM_YUAN_PLUS_DMI_ATTO3:
                    ret.safetyConfigs[0].safetyParam |= BydSafetyFlags.YUAN_PLUS_DMI_ATTO3.value

        if candidate in MPC_ACC_CAR:
            ret.networkLocation = NetworkLocation.fwdCamera

        use_torque_lat = candidate in TORQUE_LAT_CAR

        if use_torque_lat:
            CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
        else:
            ret.lateralTuning.init('pid')
            ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[8.3 , 27.8], [8.3 , 27.8]]
            ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV   = [[0.6 ,  0.3], [0.2 ,  0.1]]
            ret.lateralTuning.pid.kf = 0.000072

        use_experimental_long = candidate in EXP_LONG_CAR

        ret.alphaLongitudinalAvailable = use_experimental_long
        ret.openpilotLongitudinalControl = experimental_long and ret.alphaLongitudinalAvailable

        ret.longitudinalTuning.kpBP, ret.longitudinalTuning.kiBP = [[0.],  [0.]]
        ret.longitudinalTuning.kpV,  ret.longitudinalTuning.kiV  = [[1.5], [0.3]]

        # model specific parameters
        # Todo: Developers please fill or add more models.
        if candidate in (CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM, CAR.BYD_SONG_PLUS_DMI_21):
            ret.minSteerSpeed = 0
            ret.autoResumeSng = True
            ret.startingState = True
            ret.startAccel = 0.8
            ret.stopAccel = -0.5
            ret.vEgoStarting = 0.2 * CV.KPH_TO_MS
            ret.vEgoStopping = 0.1 * CV.KPH_TO_MS
            ret.longitudinalActuatorDelay = 0.5
        else:
            ret.dashcamOnly = True

        return ret


# byd tuning suggestions
# torque mode (linear):
#   -Start with P = 1.0 and I = 0, FRICTION = 0, change the LAT_ACCEL_FACTOR until
#    vehicle can just about to go thought a medium curve at medium speed(say 50kph)
#   -Find a straight road and tune FRICTION until car stay in same position of lane,
#    don't worry if car hugs to left or right.
#   -I = 0.1 (default) then try. Keep tuning all values until you get a good enough
#    result. Commaai's PI loop have a funny anti-wind up method.
#   -Add the steering angle speed (angle slew rate) to stop the control loop from
#    tuning wheel too sharp.
