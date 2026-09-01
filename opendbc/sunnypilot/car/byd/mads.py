"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

ButtonType = structs.CarState.ButtonEvent.Type


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.main_cruise_button = 0

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    self.prev_lkas_button = self.lkas_button
    # BYD does not expose a dedicated LKAS button on the steering wheel.
    # The ACC main switch is reused to report cruise availability; treat it
    # as the MADS toggle input so sunnypilot's MADS state machine can latch.
    self.lkas_button = cp.vl["PCM_BUTTONS"]["BTN_TOGGLE_ACC_OnOff"]

    # If desired, map rising/falling edges of the ACC main switch to a
    # mainCruise button event. Stock CarState already emits cancel/resume/gap
    # events from PCM_BUTTONS; this only adds MADS-specific latching.
    if self.lkas_button != self.prev_lkas_button:
      ret.buttonEvents.append(structs.CarState.ButtonEvent(
        pressed=bool(self.lkas_button),
        type=ButtonType.mainCruise,
      ))
