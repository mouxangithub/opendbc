from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.proton.protoncan import create_steering_control, send_buttons
from opendbc.car.proton.values import CarControllerParams
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CP = CP
    self.frame = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.num_cruise_btn_sent = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators

    # TODO laneActive, used to check if ALC is off
    lat_active = enabled and not CS.lkaDisabled

    # TODO what is this for?
    if self.frame <= 1000 and CS.out.cruiseState.available and self.num_cruise_btn_sent <= 5:
      self.num_cruise_btn_sent += 1
      can_sends.append(send_buttons(self.packer, self.frame % 16, True))

    # steer
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    # TODO use openpilot's ready function

    if not lat_active and CS.stock_ldp: # Lane Departure Prevention
      steer_dir = -1 if CS.steer_dir else 1
      new_steer = CS.stock_ldp_cmd * steer_dir * 0.0002 # Reduce value because stock command was strong
      lat_active = True

    apply_steer = apply_driver_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorque, self.params)

    # CAN controlled lateral running at 500hz
    if (self.frame % 2) == 0:
      can_sends.append(create_steering_control(self.packer, apply_steer, \
      lat_active, CS.hand_on_wheel_warning and CS.is_icc_on, (self.frame/2) % 16, \
      CS.stock_lks_settings,  CS.stock_lks_settings2))

    if CS.out.standstill and enabled and (self.frame % 29 == 0):
      # Spam resume button to resume from standstill at max freq of 34.48 Hz.
      if CS.acc_req:
        can_sends.append(send_buttons(self.packer, self.frame % 16, False))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    self.frame += 1
    return new_actuators, can_sends
