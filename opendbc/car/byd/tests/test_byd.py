import unittest

from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car import structs
from opendbc.car.structs import CarParams
from opendbc.car.byd.interface import CarInterface
from opendbc.car.byd.values import CAR, DBC, CanBus, BydSafetyFlags
from opendbc.car.byd.fingerprints import FINGERPRINTS, FW_VERSIONS
from opendbc.car.byd.bydcan import create_steering_control, acc_cmd, create_fake_318
from opendbc.car.byd.carstate import CarState
from opendbc.can.packer import CANPacker

Ecu = CarParams.Ecu


class TestBydFingerprint(unittest.TestCase):
  def test_car_params(self):
    for car_model in CAR:
      fingerprint = gen_empty_fingerprint()
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert CP.brand == "byd"
      assert CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.byd
      assert CP.carFingerprint == car_model

  def test_safety_param_flags(self):
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      expected_flag = 0
      if car_model in (CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DM):
        expected_flag = BydSafetyFlags.HAN_TANG_DMEV
      elif car_model == CAR.BYD_TANG_DMI_21:
        expected_flag = BydSafetyFlags.TANG_DMI
      elif car_model in (CAR.BYD_SONG_PLUS_DMI_21, CAR.BYD_SONG_PLUS_DMI_22, CAR.BYD_SONG_PLUS_DMI_23, CAR.BYD_SONG_PRO_DMI_22):
        expected_flag = BydSafetyFlags.SONG_PLUS_DMI
      elif car_model == CAR.BYD_QIN_PLUS_DMI_23:
        expected_flag = BydSafetyFlags.QIN_PLUS_DMI
      elif car_model == CAR.BYD_YUAN_PLUS_DMI_22:
        expected_flag = BydSafetyFlags.YUAN_PLUS_DMI_ATTO3

      assert CP.safetyConfigs[0].safetyParam & expected_flag == expected_flag, f"{car_model}: missing safety flag"

  def test_fingerprint_present(self):
    for car_model in CAR:
      assert car_model in FINGERPRINTS, f"{car_model}: missing fingerprint"

  def test_fw_versions_placeholder(self):
    # BYD FW_VERSIONS currently only contains a placeholder; ensure it parses
    for car_model, ecus in FW_VERSIONS.items():
      for ecu, fws in ecus.items():
        assert len(fws) > 0
        for fw in fws:
          assert isinstance(fw, bytes)

  def test_dbc_map(self):
    for car_model in CAR:
      assert car_model in DBC
      assert Bus.pt in DBC[car_model]


class TestBydCan(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_params(CAR.BYD_HAN_DM_20, gen_empty_fingerprint(), [], False, False, False)
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.hud = structs.CarControl.HUDControl(leftLaneVisible=True, rightLaneVisible=True,
                                             leftLaneDepart=False, rightLaneDepart=False)

  def _make_cam_lkas(self):
    return {
      "AutoFullBeamState": 0,
      "LeftLaneState": 1,
      "LKAS_Config": 2,
      "SETME2_0x1": 1,
      "MPC_State": 0,
      "AutoFullBeam_OnOff": 0,
      "LKAS_Output": 0,
      "LKAS_Active": 0,
      "SETME3_0x0": 0,
      "TrafficSignRecognition_OnOff": 0,
      "SETME4_0x0": 0,
      "SETME5_0x1": 1,
      "RightLaneState": 1,
      "LKAS_State": 0,
      "TrafficSignRecognition_Result": 0,
      "LKAS_AlarmType": 0,
      "SETME7_0x3": 3,
    }

  def _make_esc_eps(self):
    return {
      "LKAS_Prepared": 0,
      "CruiseActivated": 0,
      "TorqueFailed": 0,
      "SETME1_0x1": 1,
      "SteerWarning": 0,
      "SteerErrorCode": 0,
      "MainTorque": 0,
      "SETME3_0x1": 1,
      "SETME4_0x3": 3,
      "SteerDriverTorque": 0,
      "SETME5_0xFF": 0xFF,
      "SETME6_0xFFF": 0xFFF,
    }

  def _make_cam_acc(self):
    return {
      "AccelCmd": 0.0,
      "ComfortBandUpper": 0.05,
      "ComfortBandLower": 0.05,
      "JerkUpperLimit": 0.5,
      "SETME1_0x1": 1,
      "JerkLowerLimit": -0.5,
      "ResumeFromStandstill": 0,
      "StandstillState": 0,
      "BrakeBehaviour": 0,
      "AccReqNotStandstill": 0,
      "AccControlActive": 0,
      "AccOverrideOrStandstill": 0,
      "EspBehaviour": 0,
      "SETME2_0xF": 0xF,
    }

  def test_create_steering_control(self):
    cam_lkas = self._make_cam_lkas()
    msg = create_steering_control(self.packer, self.CP, cam_lkas, 100, 1, 1, self.hud, 0)
    assert msg[0] == 0x316  # ACC_MPC_STATE address
    assert len(msg[1]) == 8
    assert msg[2] == CanBus.ESC

  def test_acc_cmd(self):
    cam_acc = self._make_cam_acc()
    cam_acc["Counter"] = 0
    msg = acc_cmd(self.packer, self.CP, cam_acc, 50, 1.0, 0, 0, True)
    assert msg[0] == 0x32E  # ACC_CMD address
    assert len(msg[1]) == 8
    assert msg[2] == CanBus.ESC

  def test_create_fake_318(self):
    esc_eps = self._make_esc_eps()
    msg = create_fake_318(self.packer, self.CP, esc_eps, 100, True, True, True, 0)
    assert msg[0] == 0x318  # ACC_EPS_STATE address
    assert len(msg[1]) == 8
    assert msg[2] == CanBus.MPC


if __name__ == "__main__":
  unittest.main()
