from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import (
  Buttons,
  CarControllerParams,
  MazdaFlags,
  TI_STATE,
  apply_ti_steer_torque_limits,
)

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# GEN2 ACC hold/resume frame counts at 100 Hz. Translated from the source fork's
# ControlsTimer-based logic in selfdrive/car/mazda/carcontroller.py (hold_delay=0.5s,
# hold_timer=6s, resume_timer=0.5s). opendbc has no ControlsTimer, so Timer.tick() and
# Timer.interval(2) become explicit frame math.
HOLD_DELAY_FRAMES = 50         # 0.5 s wait after stopping before applying brake hold
HOLD_DURATION_FRAMES = 600     # 6 s of brake hold once delay elapses
RESUME_DURATION_FRAMES = 50    # 0.5 s of RESUME signal to release brake


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0
    self.ti_apply_torque_last = 0
    # Instance-based params: GEN2 (e.g. MAZDA_3_2019) overrides STEER_MAX and the rate/
    # driver-allowance constants in CarControllerParams.__init__(CP); GEN1 falls back to
    # the class attrs (STEER_MAX=800). See opendbc/car/mazda/values.py.
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0          # GEN1 stock-cancel sync counter
    self.standstill_frames = 0      # GEN2 ACC hold-delay / hold-duration counter
    self.resume_until_frame = -1    # GEN2 ACC resume-window end frame

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0
    ti_apply_torque = 0

    if self.CP.flags & MazdaFlags.GEN2:
      # *** GEN2 lateral *** ------------------------------------------------------------
      # Use instance-based params so STEER_MAX picks up the GEN2 8000-count envelope.
      if CC.latActive:
        new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                        CS.out.steeringTorque, self.params)

        # Torque-interceptor clamp uses its own much smaller envelope (TI_STEER_MAX=600)
        # and rate limits. Source fork tracks this separately from the GEN2 EPS torque
        # (selfdrive/car/mazda/carcontroller.py:46-50); we preserve that split here even
        # though the GEN2 EPS_LKAS message only carries the GEN2 torque.
        if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR:
          ti_new_torque = int(round(CC.actuators.torque * self.params.TI_STEER_MAX))
          ti_apply_torque = apply_ti_steer_torque_limits(ti_new_torque, self.ti_apply_torque_last,
                                                         CS.out.steeringTorque, self.params)

      # TI fault gate: when the interceptor MCU isn't in RUN state (DISCOVER/OFF/
      # DRIVER_OVER), don't fight a faulted TI -- send zero torque on EPS_LKAS. This is
      # stricter than the source fork (which only zeroed the discarded ti_apply_steer);
      # explicit per the T11 task spec.
      if self.CP.flags & MazdaFlags.TORQUE_INTERCEPTOR and CS.ti_state != TI_STATE.RUN:
        apply_torque = 0
        ti_apply_torque = 0

      # GEN2 LKAS over EPS_LKAS (addr 0x249) on bus 1. Same message also serves the TI
      # path on GEN2 hardware: panda safety treats addr 0x249 on bus 1 as MAZDA_TI_LKAS
      # when the TORQUE_INTERCEPTOR flag is set.
      can_sends.append(mazdacan.create_steering_control_gen2(self.packer, apply_torque))

      # *** GEN2 longitudinal *** -------------------------------------------------------
      # Track standstill duration at 100 Hz so the 50 Hz ACC builder can decide hold/
      # resume. Source uses ControlsTimer.active(); we use frame counts.
      if CS.out.standstill:
        self.standstill_frames += 1
      else:
        self.standstill_frames = 0

      # ACC command at 50 Hz (every other 100 Hz tick).
      if self.frame % 2 == 0:
        hold = False
        if CS.out.standstill and self.standstill_frames > HOLD_DELAY_FRAMES:
          # Mirror source resume_timer.reset() conditions: an explicit pcm-resume
          # request (unless we're still in the planner stopping state), driver
          # override / gas press, planner starting state, or the stock ACC's own
          # RESUME bit. Any of these arms the resume window.
          resume_condition = (
            (CC.cruiseControl.resume and CC.actuators.longControlState != LongCtrlState.stopping)
            or CC.cruiseControl.override
            or CS.out.gasPressed
            or CC.actuators.longControlState == LongCtrlState.starting
            or CS.acc_values.get("RESUME", 0)
          )
          if resume_condition:
            self.resume_until_frame = self.frame + RESUME_DURATION_FRAMES
          elif self.standstill_frames < HOLD_DELAY_FRAMES + HOLD_DURATION_FRAMES:
            # Apply electric brake hold for 6 s after the 0.5 s delay. Without this the
            # car only stops momentarily and creeps forward again.
            hold = True

        resume = self.frame < self.resume_until_frame
        # When openpilot owns longitudinal (alpha_long opted in), force ACCEL_CMD to either the
        # commanded accel (longActive=True) or the inactive sentinel (longActive=False). When OPL
        # is off, op_long=False keeps the legacy stock-ACC pass-through behavior.
        can_sends.append(mazdacan.create_acc_cmd(self.packer, CS.acc_values, hold, resume,
                                                 accel=CC.actuators.accel,
                                                 op_long=self.CP.openpilotLongitudinalControl,
                                                 long_active=CC.longActive))

    else:
      # *** GEN1 path *** ---------------------------------------------------------------
      # Preserved 1:1 from upstream baseline (opendbc 0.3.1). Uses the class-level
      # CarControllerParams so STEER_MAX stays at 800.
      if CC.latActive:
        new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                        CS.out.steeringTorque, CarControllerParams)

      if CC.cruiseControl.cancel:
        # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
        # a race condition with the stock system, where the second cancel from openpilot
        # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
        # read 3 messages and most likely sync state before we attempt cancel.
        self.brake_counter = self.brake_counter + 1
        if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
          # Cancel Stock ACC if it's enabled while OP is disengaged
          # Send at a rate of 10hz until we sync with stock ACC state
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
      else:
        self.brake_counter = 0
        if CC.cruiseControl.resume and self.frame % 5 == 0:
          # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
          # Send Resume button when planner wants car to move
          can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

      # send HUD alerts
      if self.frame % 50 == 0:
        ldw = CC.hudControl.visualAlert == VisualAlert.ldw
        steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
        # TODO: find a way to silence audible warnings so we can add more hud alerts
        steer_required = steer_required and CS.lkas_allowed_speed
        can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

      # send GEN1 LKAS steering command
      can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                        self.frame, apply_torque, CS.cam_lkas))

      # Intelligent Cruise Button Management — GEN1 only.
      # GEN2 uses a different button protocol (different DBC, no CRZ_BTNS on bus 0);
      # create_button_cmd() returns None for GEN2 so ICBM would append None to can_sends.
      # Gate to GEN1 to avoid protocol mismatch.
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    self.apply_torque_last = apply_torque
    self.ti_apply_torque_last = ti_apply_torque

    new_actuators = CC.actuators.as_builder()
    # Normalize the CAN value back to the [-1, 1] actuator range. GEN2 uses the instance
    # STEER_MAX (8000); GEN1 uses the class attr (800).
    steer_max = self.params.STEER_MAX if self.CP.flags & MazdaFlags.GEN2 else CarControllerParams.STEER_MAX
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
