#!/usr/bin/env python3
from cereal import car
from opendbc.car import get_safety_config, structs
from openpilot.selfdrive.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.proton.values import CAR

class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, is_release, docs) -> structs.CarParams: # type: ignore
    ret.brand = "proton"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.proton)]

    ret.dashcamOnly = False
    ret.safetyConfigs[0].safetyParam = 1   # TODO: add actual safetyParam

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.30          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')

    ret.lateralTuning.pid.kpBP = [0., 25., 35., 40.]
    ret.lateralTuning.pid.kpV = [0.05, 0.15, 0.15, 0.16]
    ret.lateralTuning.pid.kiBP = [0., 20., 30.]
    ret.lateralTuning.pid.kiV = [0.10, 0.20, 0.40]
    ret.lateralTuning.pid.kf = 0.00007

    ret.longitudinalTuning.kpBP = [0., 5., 20.]
    ret.longitudinalTuning.kpV = [0, 0, 0]
    ret.longitudinalActuatorDelayLowerBound = 0.42
    ret.longitudinalActuatorDelayUpperBound = 0.60
    ret.longitudinalTuning.deadzoneBP = [0., 8.05]
    ret.longitudinalTuning.deadzoneV = [0, 0]
    ret.longitudinalTuning.kiBP = [0., 5., 20.]
    ret.longitudinalTuning.kiV = [0, 0, 0]

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933

    ret.openpilotLongitudinalControl = True

    if candidate == CAR.PROTON_X50:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [530]]
    elif candidate == CAR.PROTON_S70:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [530]]
    elif candidate == CAR.PROTON_X90:
      ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [530]]
    else:
      ret.dashcamOnly = True

    ret.minEnableSpeed = -1
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.001 # reach stopping target smoothly

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp)

    # events
    events = self.create_common_events(ret)

    # TODO: if self.CS.hand_on_wheel_warning and self.CS.is_icc_on:
    #  events.add(EventName.protonHandOnWheelWarning)

    ret.events = events.to_msg()
    return ret

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
