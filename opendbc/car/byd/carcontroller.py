import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams, PLATFORM_ATTO3_GENERAL

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Counter resync threshold: if steering frames exceed this, re-initialize MPC/EPS counters
MAX_STEER_FRAMES_WITHOUT_SYNC = 32  # ~640ms at 50Hz (STEER_STEP=2)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)

    self.is_atto3_general = CP.carFingerprint in PLATFORM_ATTO3_GENERAL

    self.packer = CANPacker(dbc_names[Bus.pt])

    self.frame = 0
    self.last_steer_frame = 0
    self.last_acc_frame = 0

    self.apply_torque_last = 0

    self.mpc_lkas_counter = 0
    self.eps_fake318_counter = 0

    self.lkas_req_prepare = 0
    self.lkas_active = 0

    self.steer_softstart_limit = 0
    self.steerRateLimActive = False
    self.steerRateLim = 1.0

    self.first_start = True
    self.rfss = 0  # resume from stand still
    self.sss = 0   # stand still state

    self.apply_accel_last = 0

    # ATTO3 state
    self.atto3_apply_angle_last = 0.0
    self.atto3_acc_idx = 0

  def _update_atto3(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    if self.frame % CarControllerParams.STEER_STEP == 0:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.atto3_apply_angle_last,
                                                 CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                 CC.latActive, CarControllerParams.ANGLE_LIMITS)

      if CS.out.steeringTorque > CarControllerParams.STEER_DRIVER_ALLOWANCE_ANGLE:
        apply_angle = CS.out.steeringAngleDeg

      apply_angle = float(np.clip(apply_angle,
                                  CS.out.steeringAngleDeg - CarControllerParams.MAX_ANGLE_ERROR,
                                  CS.out.steeringAngleDeg + CarControllerParams.MAX_ANGLE_ERROR))

      self.atto3_apply_angle_last = apply_angle

      if CC.latActive:
        template = CS.atto3_steer_template or bydcan.ATTO3_STEER_TEMPLATE_DEFAULT
        can_sends.append(bydcan.atto3_create_steering_control(self.packer, apply_angle, template,
                                                              self.frame // CarControllerParams.STEER_STEP))
        can_sends.append(bydcan.atto3_create_lkas_hud(self.packer, CS.atto3_lkas_hud,
                                                      self.frame // CarControllerParams.STEER_STEP))

    if self.CP.openpilotLongitudinalControl:
      if self.frame % CarControllerParams.STEER_STEP == 0:
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        if not CC.longActive or pcm_cancel_cmd:
          accel = 0.0

        can_sends.append(bydcan.atto3_create_acc_control(self.packer, accel,
                                                         CC.longActive and not pcm_cancel_cmd, self.atto3_acc_idx))

        set_speed = hud_control.setSpeed if hud_control.setSpeed > 0 else CS.out.cruiseState.speed
        can_sends.append(bydcan.atto3_create_acc_hud(self.packer, CC.enabled, set_speed * 3.6,
                                                     hud_control.leadVisible, self.atto3_acc_idx))
        self.atto3_acc_idx += 1

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.atto3_apply_angle_last

    self.frame += 1
    return new_actuators, can_sends

  def _update_legacy(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # Check if we need to re-sync counters (e.g., after communication interruption)
    frames_since_steer = self.frame - self.last_steer_frame
    if frames_since_steer > MAX_STEER_FRAMES_WITHOUT_SYNC:
      self.first_start = True

    if (self.frame - self.last_steer_frame) >= CarControllerParams.STEER_STEP:

      # Resolve counter mismatch problem
      if self.first_start:
        # Counter initialization with proper wrap-around
        self.mpc_lkas_counter = (CS.acc_mpc_state_counter + 1) % 16
        self.eps_fake318_counter = (CS.eps_state_counter + 1) % 16
        self.first_start = False

      apply_torque = 0

      if CC.latActive:
        if self.lkas_active:
          steer_desire = CC.actuators.torque

          if CarControllerParams.USE_STEERING_SPEED_LIMITER:  # Use steering angular speed limiter
            rate_limit = np.interp(CS.out.aEgo, [8.3, 27.8], [132, 64])
            delta_rate = CS.steeringRateDegAbs - rate_limit

            if delta_rate < 0:
              self.steerRateLim -= 0.005 * delta_rate

              if delta_rate < -0.05:
                self.steerRateLimActive = False

              if self.steerRateLim > 1.0:
                self.steerRateLim = 1.0
                self.steerRateLimActive = False

            else:
              if self.steerRateLimActive:
                self.steerRateLim -= 0.005 * delta_rate
              else:
                self.steerRateLim = steer_desire
                self.steerRateLimActive = True

              if self.steerRateLim < 0:
                self.steerRateLim = 0

            new_steer_pu = np.clip(steer_desire, -self.steerRateLim, self.steerRateLim)
          else:
            new_steer_pu = steer_desire

          new_steer = int(round(new_steer_pu * CarControllerParams.STEER_MAX))

          if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
            self.steer_softstart_limit = self.steer_softstart_limit + CarControllerParams.STEER_SOFTSTART_STEP
            new_steer = np.clip(new_steer, -self.steer_softstart_limit, self.steer_softstart_limit)

          apply_torque = apply_driver_steer_torque_limits(new_steer, self.apply_torque_last,
                                                        CS.out.steeringTorque, CarControllerParams)

        else:
          if CS.lkas_prepared:
            self.lkas_active = 1
            self.steerRateLimActive = False
            self.steerRateLim = 1.0
            self.lkas_req_prepare = 0
            self.steer_softstart_limit = 0
          else:
            self.lkas_req_prepare = 1

      else:
        self.lkas_req_prepare = 0
        self.steerRateLimActive = False
        self.steerRateLim = 1.0
        self.lkas_active = 0
        self.steer_softstart_limit = 0

      self.apply_torque_last = apply_torque

      self.mpc_lkas_counter = (self.mpc_lkas_counter + 1) % 16
      self.eps_fake318_counter = (self.eps_fake318_counter + 1) % 16
      self.last_steer_frame = self.frame

      # send steering command, op to esc
      can_sends.append(bydcan.create_steering_control(self.packer, self.CP, CS.cam_lkas,
          self.apply_torque_last, self.lkas_req_prepare, self.lkas_active, CC.hudControl, self.mpc_lkas_counter))

      # send fake 318 from op to mpc
      can_sends.append(bydcan.create_fake_318(self.packer, self.CP, CS.esc_eps,
                                               CS.mpc_laks_output, CS.mpc_laks_reqprepare, CS.mpc_laks_active,
                                               True, self.eps_fake318_counter))

    if (self.frame + 1 - self.last_acc_frame) >= CarControllerParams.ACC_STEP:
      accel = np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      if CC.longActive:
        stopping = CC.actuators.longControlState == LongCtrlState.stopping
        starting = CC.actuators.longControlState == LongCtrlState.starting
        running = CC.actuators.longControlState == LongCtrlState.pid

        # stopping and stopped
        if stopping and accel < -0.1:
          self.rfss = 0
          self.sss = CS.out.standstill

        # re-starting
        elif starting and accel > 0.1 and CS.out.vEgo < 0.8:
          self.rfss = CS.out.standstill
          self.sss = 0

        # started
        elif running:
          self.rfss = 0
          self.sss = 0

      else:
        accel = 0
        self.sss = 0
        self.rfss = 0

      can_sends.append(bydcan.acc_cmd(self.packer, self.CP, CS.cam_acc, CS.mrr_leading_dist, accel, self.rfss, self.sss, CC.longActive))

      self.apply_accel_last = accel
      self.last_acc_frame = self.frame + 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = float(self.apply_accel_last)
    new_actuators.steeringAngleDeg = float(CS.out.steeringAngleDeg)

    self.frame += 1
    return new_actuators, can_sends

  def update(self, CC, CC_SP, CS, now_nanos):
    if self.is_atto3_general:
      return self._update_atto3(CC, CC_SP, CS, now_nanos)
    return self._update_legacy(CC, CC_SP, CS, now_nanos)
