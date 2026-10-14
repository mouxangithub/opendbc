"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import DT_CTRL, rate_limit
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams

# Stock body-latched releases use a RESUME_UNLATCHING pulse of 9 wire frames, the latched-family mode.
RESUME_UNLATCH_LATCHED_FRAMES = int(0.18 / DT_CTRL)
# Retry one unanswered body-latched release after this, the body still holding, then return control to the plan.
RESUME_REPULSE_FRAMES = int(1.0 / DT_CTRL)
# Debounce lead visibility before advertising a radar track.
LEAD_DEBOUNCE_FRAMES = int(0.5 / DT_CTRL)
# Debounce movement requests before releasing a standstill hold.
RELEASE_DEBOUNCE_FRAMES = int(0.2 / DT_CTRL)
# A plan must ask for more than this to open a hold: the e2e model drifts up to +0.19 at a stop
# before its shouldStop lands, and every logged drive-off passes 0.25 within 1.1 s.
RELEASE_ACCEL = 0.25  # m/s^2
# The still-stopped breakaway ramp gives up after this.
BREAKAWAY_FRAMES = int(3.0 / DT_CTRL)


class StandstillHold:
  """Hold the car until the plan or driver requests movement.

  The plan supplies the hold command. It relaxes when the body ECU takes brake ownership,
  matching the stock stop-and-go sequence.
  """

  def __init__(self):
    self._reset()

  def _reset(self):
    self.holding = False
    self.car_has_hold = False
    self.unlatch_frames = 0
    self.release_frames = 0
    self.latched_release = False
    self.just_released = False
    self.latched_frames = 0  # frames waiting for the body ECU to release its hold
    self.repulsed = False

  def update(self, long_engaged: bool, stopping: bool, standstill: bool,
             plan_accel: float, body_hold: bool, gas_pressed: bool) -> None:
    self.just_released = False
    if not long_engaged:
      self._reset()
      return

    was_holding = self.holding
    # Debounce plan movement requests: opening a hold takes RELEASE_ACCEL, keeping it open any
    # positive plan. Driver throttle releases the hold immediately.
    release_accel = RELEASE_ACCEL if was_holding else 0.
    self.release_frames = self.release_frames + 1 if plan_accel > release_accel else 0
    plan_wants_go = self.release_frames >= RELEASE_DEBOUNCE_FRAMES
    # Keep STOPPING off once the plan or driver requests acceleration.
    release = gas_pressed or plan_wants_go
    self.holding = not release and (stopping or standstill)

    if self.unlatch_frames > 0:
      self.unlatch_frames -= 1
    # Send one unlatch pulse per plan-driven body-latched release. Throttle releases use none.
    if was_holding and not self.holding and standstill and not gas_pressed and self.unlatch_frames == 0:
      # The previous frame records whether the body owned a hold that must be unlatched.
      self.latched_release = self.car_has_hold
      if self.latched_release:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
      self.just_released = True
      self.latched_frames = 0
      self.repulsed = False

    # Retry one unanswered body-latched release so a positive plan cannot remain blocked.
    if self.latched_release and not self.holding and standstill and body_hold and not gas_pressed:
      self.latched_frames += 1
      if self.latched_frames >= RESUME_REPULSE_FRAMES and not self.repulsed and self.unlatch_frames == 0:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
        self.repulsed = True
    else:
      self.latched_frames = 0

    # Body ownership is valid only while the controller still requests a hold.
    self.car_has_hold = self.holding and standstill and body_hold

  @property
  def stop_bits(self) -> bool:
    # Keep STOPPING and RESUME_UNLATCHING mutually exclusive, including during a re-hold.
    return self.holding and not self.car_has_hold and self.unlatch_frames == 0

  @property
  def resume_unlatching(self) -> bool:
    return self.unlatch_frames > 0

  @property
  def acc_active_2(self) -> bool:
    # Stock clears ACC_ACTIVE_2 when the command relaxes.
    return not self.car_has_hold


class AccelShaper:
  """The CRZ_INFO command between the plan and the wire, shaped like stock MRCC's.

  A hold's release ramps up from the relaxed command, with a bounded breakaway while the car is
  still stopped. Otherwise the plan is slew-limited, and positive commands follow stock's
  ceiling and build rate at this speed and lift at stock's rate. The standstill hold's own
  commands come last. accel_last is the command sent the frame before.
  """

  def __init__(self):
    self.release_ramp: float | None = None
    self.breakaway_frames = 0

  def update(self, long_active: bool, plan_accel: float, accel_last: float, sm: StandstillHold,
             standstill: bool, body_hold: bool, v_ego: float) -> float:
    if sm.just_released:
      # Never-latched stops relax in one frame; latched holds ramp from the relaxed command.
      self.release_ramp = CarControllerParams.ACCEL_HOLD_LATCHED if sm.latched_release else \
                          CarControllerParams.ACCEL_RELEASE_BAND
    elif sm.holding or not long_active:
      # Re-holds and driver overrides terminate the release ramp.
      self.release_ramp = None

    accel = 0.
    if long_active:
      accel = float(np.clip(plan_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      # Continue a bounded release ramp while stopped because the plan may not break static hold.
      if self.release_ramp is None or not standstill:
        self.breakaway_frames = 0
      else:
        self.breakaway_frames += 1
      breakaway = standstill and self.breakaway_frames <= BREAKAWAY_FRAMES
      # Bound breakaway by stock authority and by the plan-relative margin.
      ramp_ceiling = max(accel, min(CarControllerParams.ACCEL_BREAKAWAY_MAX,
                                    accel + CarControllerParams.ACCEL_BREAKAWAY_OVERSHOOT))
      if self.release_ramp is not None and (self.release_ramp < accel or breakaway):
        # The release ramp owns the command until it reaches the plan. Body-latched holds remain
        # at the relaxed command until the body lets go.
        accel = self.release_ramp
        if not (sm.latched_release and body_hold):
          # Follow a falling plan ceiling at the winddown limit.
          self.release_ramp = max(min(self.release_ramp + CarControllerParams.ACCEL_RELEASE_RAMP * DT_CTRL, ramp_ceiling),
                                  self.release_ramp + CarControllerParams.ACCEL_WINDDOWN_LIMIT)
      else:
        self.release_ramp = None
        # Track overrides in accel_last so control resumes through the slew limiter.
        accel = rate_limit(accel, accel_last, CarControllerParams.ACCEL_WINDDOWN_LIMIT,
                           CarControllerParams.ACCEL_WINDUP_LIMIT)
        if accel > 0.:
          # Shape positive commands to stock MRCC's ceiling and build rate at this speed.
          ceiling = float(np.interp(v_ego, CarControllerParams.ACCEL_CEILING_BP, CarControllerParams.ACCEL_CEILING_V))
          build = float(np.interp(v_ego, CarControllerParams.ACCEL_BUILD_BP, CarControllerParams.ACCEL_BUILD_V)) * DT_CTRL
          accel = min(accel, ceiling, max(accel_last, 0.) + build)
        if accel_last > 0. and plan_accel >= 0.:
          # Lift the throttle at stock's rate; a brake request bypasses this above.
          accel = max(accel, accel_last + CarControllerParams.ACCEL_LIFT_LIMIT * DT_CTRL)
      if sm.car_has_hold:
        # Stop requesting brake hold after the body ECU takes ownership.
        accel = CarControllerParams.ACCEL_HOLD_LATCHED
      elif sm.holding:
        # Freeze the braking command while STOPPING is asserted.
        accel = min(accel, 0.) if plan_accel <= 0. else min(accel_last, 0.)
      if sm.resume_unlatching:
        # Bound the latched release pulse to stock's command range.
        accel = min(max(accel, CarControllerParams.ACCEL_HOLD_LATCHED),
                    CarControllerParams.ACCEL_RESUME_PULSE_MAX)
    return accel


class AdvertisedLead:
  """Maintain consistent lead state across CRZ_CTRL and the 0x364 track.

  Advertisement follows perception rather than engagement. Visibility is debounced and the
  last measurement is propagated through short vision gaps like a radar track.
  """

  def __init__(self):
    self.visible = False
    self.flip_frames = 0
    self.holding = False
    self.lead = None
    self._measured = None

  def update(self, lead_visible: bool, d_rel: float, v_rel: float, holding: bool) -> None:
    if lead_visible != self.visible:
      self.flip_frames += 1
      if self.flip_frames >= LEAD_DEBOUNCE_FRAMES:
        self.visible = lead_visible
        self.flip_frames = 0
    else:
      self.flip_frames = 0

    if 0. < d_rel <= mazdacan.DIST_OBJ_MAX:
      self._measured = (d_rel, v_rel)
    elif not self.visible:
      # Expire stale state after the debounce window.
      self._measured = None
    elif self._measured is not None:
      # Propagate range through the gap instead of repeating a frozen track.
      d, v = self._measured
      self._measured = (d + v * DT_CTRL, v)
    self.lead = self._measured if self.visible else None
    self.holding = holding

  @property
  def has_lead(self) -> bool:
    return self.lead is not None

  @property
  def ctrl_phase(self) -> int:
    # Use stock's relative-distance buckets and keep all lead fields consistent.
    if not self.has_lead:
      return 0
    return 3 if self.holding else 2
