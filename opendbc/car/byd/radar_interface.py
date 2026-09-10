#!/usr/bin/env python3
import math

from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.byd.byd_params import BydParams
from opendbc.car.byd.values import CanBus, PT_RADAR_100, PT_RADAR_CAR, RADAR_CAR


RADAR_START_ADDR = 0x351
RADAR_MSG_COUNT = 4
RADAR_HEADER_MSG = 0x460
SLOT_1_MSG = 0x461
LAST_RADAR_MSG = 0x474
NUM_SLOTS = 20


def create_gm_radar_can_parser():
  msg_ids = list(range(SLOT_1_MSG, SLOT_1_MSG + NUM_SLOTS))
  signals = list(zip(
    ["FLRRNumValidTargets", "FLRRSnsrBlckd", "FLRRYawRtPlsblityFlt", "FLRRHWFltPrsntInt",
     "FLRRAntTngFltPrsnt", "FLRRAlgnFltPrsnt", "FLRRSnstvFltPrsntInt"] +
    ["TrkRange"] * NUM_SLOTS + ["TrkRangeRate"] * NUM_SLOTS + ["TrkRangeAccel"] * NUM_SLOTS +
    ["TrkAzimuth"] * NUM_SLOTS + ["TrkWidth"] * NUM_SLOTS + ["TrkObjectID"] * NUM_SLOTS,
    [RADAR_HEADER_MSG] * 7 + msg_ids * 6,
    strict=True,
  ))
  checks = list({(msg, 6) for _, msg in signals})
  return CANParser("gm_global_a_object", checks, CanBus.MRR)


def get_radar_can_parser(CP, radar_tracks, msg_start_addr, msg_count):
  if not radar_tracks:
    return None
  checks = [(f"RADAR_TRACK_{addr:x}", 25) for addr in range(msg_start_addr, msg_start_addr + msg_count)]
  return CANParser("byd_tangdm_radar", checks, CanBus.MRR)


def get_pt_radar_can_parser(CP, radar_tracks, freq):
  if not radar_tracks:
    return None
  return CANParser("byd_generic_pt", [("CID_NCBHOL", freq)], CanBus.MPC)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.CP = CP
    self.params = BydParams()
    self.radar_tracks = self.params.get_int("EnableRadarTracks") >= 1
    self.pt_type = 0
    self.use_ext_radar = self.params.get_bool("EnableExtRadar")

    if self.use_ext_radar:
      self.updated_tracks = set()
      self.rcp = create_gm_radar_can_parser()
      self.trigger_msg = LAST_RADAR_MSG
      print("RadarInterface: Use pt radar...")
    elif self.CP.carFingerprint in PT_RADAR_CAR:
      self.updated_tracks = set()
      if self.CP.carFingerprint in PT_RADAR_100:
        self.rcp = get_pt_radar_can_parser(CP, self.radar_tracks, 100)
        self.pt_type = 1
      else:
        self.rcp = get_pt_radar_can_parser(CP, self.radar_tracks, 60)
        self.pt_type = 0
      self.trigger_msg = 884
      print("RadarInterface: Use pt radar...")
    elif self.CP.carFingerprint in RADAR_CAR:
      self.radar_start_addr = RADAR_START_ADDR
      self.radar_msg_count = RADAR_MSG_COUNT
      self.updated_tracks = set()
      self.rcp = get_radar_can_parser(CP, self.radar_tracks, self.radar_start_addr, self.radar_msg_count)
      self.trigger_msg = self.radar_start_addr + self.radar_msg_count - 1
      print("RadarInterface: Use radar...")
    else:
      self.rcp = None
      print("RadarInterface: No radar...")

  def update(self, can_strings):
    if self.rcp is None or self.CP.radarUnavailable:
      return super().update(None)
    if not self.radar_tracks:
      return None

    updated = self.rcp.update(can_strings)
    self.updated_tracks.update(updated)
    if self.trigger_msg not in self.updated_tracks:
      return None

    if self.use_ext_radar:
      self._update_ext(self.updated_tracks)
    elif self.CP.carFingerprint in PT_RADAR_CAR:
      self._update_pt(self.updated_tracks)
    elif self.CP.carFingerprint in RADAR_CAR:
      self._update(self.updated_tracks)
    self.updated_tracks.clear()

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
    ret.points = list(self.pts.values())
    return ret

  def _update_ext(self, updated_tracks):
    header = self.rcp.vl[RADAR_HEADER_MSG]
    valid_ids = set()
    valid_target_count = int(header["FLRRNumValidTargets"])
    for msg in updated_tracks:
      if msg == RADAR_HEADER_MSG:
        continue
      if valid_target_count <= 0:
        break
      track = self.rcp.vl[msg]
      d_rel = float(track["TrkRange"])
      track_id = int(track["TrkObjectID"])
      azimuth = float(track["TrkAzimuth"])
      if not (0.1 < d_rel < 200.0):
        continue
      if track_id in (0, 255):
        continue
      if abs(azimuth) > 45.0:
        continue
      valid_ids.add(track_id)
      if track_id not in self.pts:
        self.pts[track_id] = structs.RadarData.RadarPoint()
        self.pts[track_id].trackId = track_id
      pt = self.pts[track_id]
      pt.dRel = d_rel
      pt.yRel = math.sin(azimuth * CV.DEG_TO_RAD) * d_rel
      pt.vRel = float(track["TrkRangeRate"])
      pt.vLead = pt.vRel + self.v_ego
      pt.aRel = float("nan")
      pt.yvRel = 0.0
      pt.measured = True
    for track_id in list(self.pts.keys()):
      if track_id not in valid_ids:
        del self.pts[track_id]

  def _update_pt(self, updated_tracks):
    track = self.rcp.vl["CID_NCBHOL"]
    track_id = track["sig_qfnwgn"]
    d_rel = track["sig_voxvab"]
    if self.pt_type == 0:
      valid = d_rel > 0 or bool(track["sig_ijbmoi"])
    else:
      valid = d_rel > 0 or bool(track["sig_bzpoue"])

    if valid:
      if track_id not in self.pts:
        self.pts[track_id] = structs.RadarData.RadarPoint()
        self.pts[track_id].trackId = track_id
      pt = self.pts[track_id]
      pt.dRel = d_rel
      pt.yRel = track["sig_lhpiui"]
      pt.vRel = float("nan")
      pt.aRel = float("nan")
      pt.yvRel = float("nan")
      pt.measured = True
    elif track_id in self.pts:
      del self.pts[track_id]

  def _update(self, updated_tracks):
    point_id = 0
    for addr in range(self.radar_start_addr, self.radar_start_addr + self.radar_msg_count):
      track = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]
      valid = bool(track["IsValid_tang"])
      if point_id not in self.pts:
        self.pts[point_id] = structs.RadarData.RadarPoint()
      pt = self.pts[point_id]
      pt.trackId = point_id
      pt.measured = valid
      if valid:
        pt.dRel = track["LongDist_tang"]
        pt.yRel = track["LatDist_tang"]
        pt.vRel = track["RelSpeed_tang"]
        pt.aRel = float("nan")
        pt.yvRel = float("nan")
      else:
        pt.dRel = 0
        pt.yRel = 0
        pt.vRel = 0
        pt.aRel = float("nan")
        pt.yvRel = 0
      point_id += 1
