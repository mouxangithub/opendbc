"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The MADS white wheel: with lateral active on a TJA-declared car and cruise completely
off, the TJA=2 state is XORed into the camera's own allowlisted idle frame. Under test:
an undeclared car is byte-identical to the base behavior, every fail-closed deny holds,
and an unsafe state already on the bus is withdrawn immediately, outside the 2 Hz cadence.
"""

import pytest

from opendbc.car.mazda import mazdacan
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

from opendbc.car.mazda.tests.conftest import (CAM_LANEINFO, SendButtonState, VisualAlert, car_controller,
                                               mazda_car_state, packer, step)

BASE = bytes.fromhex("4201000000001040")  # the canonical OFF-family idle base
CADENCE = 50                              # the normal 2 Hz alert slot


def hud_frames(sends) -> list[bytes]:
  return [d for a, d, b in sends if a == CAM_LANEINFO and b == 0]


def tja_controller(alpha_long=True):
  cc = car_controller(alpha_long=alpha_long)
  cc.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  return cc, cs


def drive(cc, cs, cycles, **kwargs):
  hud, sends = [], []
  for _ in range(cycles):
    _, out = step(cc, cs, **kwargs)
    hud.extend((cc.frame - 1, d) for d in hud_frames(out))
    sends.extend(out)
  return hud, sends


class TestShipsDark:
  def test_undeclared_car_is_byte_identical_and_on_cadence(self):
    cc, cs = tja_controller()
    cc.CP_SP.flags &= ~MazdaFlagsSP.TJA_BUTTON
    hud, _ = drive(cc, cs, 2 * CADENCE + 5, mads_active=True, cam_laneinfo_raw=BASE,
                   cam_laneinfo_live=True, available=False, mrcc_armed_raw=False)
    assert hud, "the alert cadence itself must not change"
    assert all(frame % CADENCE == 0 for frame, _ in hud)
    assert not any(mazdacan.is_mads_white_hud(d) for _, d in hud)
    expected = mazdacan.create_alert_command(packer(), cs.cam_laneinfo, False, False)[1]
    assert all(d == expected for _, d in hud)


class TestWhiteWheelGate:
  def kwargs(self, **over):
    kw = dict(mads_active=True, cam_laneinfo_raw=BASE, cam_laneinfo_live=True,
              available=False, mrcc_armed_raw=False)
    kw.update(over)
    return kw

  def test_white_wheel_displays_on_the_allowlisted_base(self):
    cc, cs = tja_controller()
    hud, _ = drive(cc, cs, 2 * CADENCE + 5, **self.kwargs())
    whites = [d for _, d in hud if mazdacan.is_mads_white_hud(d)]
    assert whites, "the white wheel must display once confirmed"
    assert all(d == bytes.fromhex("4201000020001040") for d in whites)
    assert cc.mads_white_hud_on_bus

  def test_confirmation_delay_before_display(self):
    cc, cs = tja_controller()
    hud, _ = drive(cc, cs, CADENCE, **self.kwargs())
    # the frame-0 slot is inside the 0.5 s confirmation window: base frame, no white
    assert hud and not any(mazdacan.is_mads_white_hud(d) for _, d in hud)

  @pytest.mark.parametrize("deny,over", [
    ("MADS off", dict(mads_active=False)),
    ("cruise armed", dict(mrcc_armed_raw=True)),
    ("cruise available", dict(available=True)),
    ("stock ECU hand-back", dict(handback=True)),
    ("stale camera frame", dict(cam_laneinfo_live=False)),
    ("unknown camera payload", dict(cam_laneinfo_raw=b"\xff" * 8)),
    ("no camera payload", dict(cam_laneinfo_raw=None)),
    ("wheel button held", dict(tja_button=1)),
    ("MRCC button held", dict(mrcc_button=1)),
    ("ICBM set button", dict(send_button=SendButtonState.increase)),
    ("visual alert", dict(visual_alert=VisualAlert.steerRequired)),
  ])
  def test_every_deny_blocks_the_white_state(self, deny, over):
    cc, cs = tja_controller()
    hud, _ = drive(cc, cs, 2 * CADENCE + 5, **self.kwargs(**over))
    assert not any(mazdacan.is_mads_white_hud(d) for _, d in hud), deny

  def test_radar_handback_blocks_the_white_state(self):
    # under alpha-long the session manager owns radar_handback_active and overwrites a
    # seeded flag, so the deny is exercised on a stock-long controller
    cc, cs = tja_controller(alpha_long=False)
    hud, _ = drive(cc, cs, 2 * CADENCE + 5, **self.kwargs(radar_handback_active=True))
    assert not any(mazdacan.is_mads_white_hud(d) for _, d in hud)

  def test_unsafe_state_is_withdrawn_immediately(self):
    cc, cs = tja_controller()
    hud, _ = drive(cc, cs, 2 * CADENCE + 5, **self.kwargs())
    assert cc.mads_white_hud_on_bus

    # cruise arms mid-display: the white bit leaves the bus on the very next cycle,
    # not at the next 2 Hz slot
    withdrawn, _ = drive(cc, cs, 3, **self.kwargs(mrcc_armed_raw=True))
    frames_out = [f for f, d in withdrawn]
    assert frames_out, "the withdraw frame must go out at once"
    assert all(f % CADENCE != 0 for f in frames_out)
    assert all(not mazdacan.is_mads_white_hud(d) for _, d in withdrawn)
    assert not cc.mads_white_hud_on_bus
