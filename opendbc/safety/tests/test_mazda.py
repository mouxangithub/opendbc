#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg

FLAG_MAZDA_GEN2 = 2
FLAG_MAZDA_TORQUE_INTERCEPTOR = 8
FLAG_MAZDA_LONG = 16

MAZDA_MAIN = 0
MAZDA_AUX = 1
MAZDA_CAM = 2

MAZDA_2019_ACC = 0x220
MAZDA_TI_LKAS = 0x249


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))


class TestMazdaGen2Safety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  FLAGS = FLAG_MAZDA_GEN2
  TX_MSGS = [[MAZDA_TI_LKAS, MAZDA_AUX], [MAZDA_2019_ACC, MAZDA_CAM]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {MAZDA_AUX: (MAZDA_TI_LKAS,), MAZDA_CAM: (MAZDA_2019_ACC,)}
  FWD_BLACKLISTED_ADDRS = {MAZDA_MAIN: [MAZDA_2019_ACC, MAZDA_TI_LKAS], MAZDA_CAM: [MAZDA_TI_LKAS]}

  MAX_RATE_UP = 45
  MAX_RATE_DOWN = 80
  MAX_TORQUE_LOOKUP = [0], [8000]

  MAX_RT_DELTA = 1688

  DRIVER_TORQUE_ALLOWANCE = 1400
  DRIVER_TORQUE_FACTOR = 1

  # Mazda TI LKAS does not use a separate request bit.
  NO_STEER_REQ_BIT = True
  DRIVER_TORQUE_BUS = MAZDA_AUX

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2019")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.FLAGS)
    self.safety.init_tests()

  @staticmethod
  def _signed_word_msg(addr, bus, value):
    return libsafety_py.make_CANPacket(addr, bus, int(value).to_bytes(2, "big", signed=True) + b"\x00" * 6)

  def _torque_meas_msg(self, torque):
    return self._torque_driver_msg(torque)

  def _torque_driver_msg(self, torque):
    return self._signed_word_msg(0x24b, self.DRIVER_TORQUE_BUS, torque)

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._signed_word_msg(MAZDA_TI_LKAS, MAZDA_AUX, torque)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("SPEED", MAZDA_CAM, values)

  def _speed_msg_2(self, speed: float):
    return None

  def _wheel_speeds_msg(self, speed):
    values = {s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", MAZDA_CAM, values)

  def _user_brake_msg(self, brake):
    dat = bytearray(8)
    dat[5] = 0x4 if brake else 0x0
    return libsafety_py.make_CANPacket(0x43f, MAZDA_MAIN, bytes(dat))

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", MAZDA_CAM, values)

  def _pcm_status_msg(self, enable):
    dat = bytearray(8)
    dat[0] = 0x20 if enable else 0x0
    return libsafety_py.make_CANPacket(0x44a, MAZDA_MAIN, bytes(dat))

  def _button_msg(self):
    values = {"CAN": 0, "RES": 0, "SET_M": 0, "SET_P": 0}
    return self.packer.make_can_msg_safety("CRZ_BTNS", MAZDA_MAIN, values)

  def _send_valid_gen2_rx(self, speed_msg):
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertTrue(self._rx(speed_msg))
    self.assertTrue(self._rx(self._torque_driver_msg(0)))
    self.assertTrue(self._rx(self._button_msg()))

  def test_speed_rx_checks_accept_speed_msg(self):
    self._send_valid_gen2_rx(self._speed_msg(1.0))
    self.safety.set_timer(1000)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def test_speed_rx_checks_accept_wheel_speeds_msg(self):
    self._send_valid_gen2_rx(self._wheel_speeds_msg(1.0))
    self.safety.set_timer(1000)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def test_gen2_acc_tx_allowed(self):
    self.assertTrue(self._tx(libsafety_py.make_CANPacket(MAZDA_2019_ACC, MAZDA_CAM, b"\x00" * 8)))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(MAZDA_2019_ACC, MAZDA_MAIN, b"\x00" * 8)))


class TestMazdaGen2TiSafety(TestMazdaGen2Safety):
  FLAGS = FLAG_MAZDA_GEN2 | FLAG_MAZDA_TORQUE_INTERCEPTOR
  DRIVER_TORQUE_BUS = MAZDA_MAIN

  def test_ti_lkas_is_only_allowed_bus_1_tx(self):
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))

    for addr in [0x220, 0x243, 0x440, 0x24b, 0x74b, 0x74c]:
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(addr, MAZDA_AUX, b"\x00" * 8)))

  def test_bus_0_driver_torque_blocks_ti_lkas(self):
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(-self.DRIVER_TORQUE_ALLOWANCE - 1)
    self._set_prev_torque(self.MAX_TORQUE)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE)))

  def test_aux_bus_not_forwarded(self):
    for addr in [0, MAZDA_TI_LKAS, 0x220, 0x7ff]:
      self.assertEqual(-1, self.safety.safety_fwd_hook(MAZDA_AUX, addr))


class TestMazdaGen2LongSafety(TestMazdaGen2Safety, common.LongitudinalAccelSafetyTest):
  # FLAG_MAZDA_LONG enables ACCEL_CMD validation against MAZDA_2019_LONG_LIMITS.
  # Mixin defaults (MAX_ACCEL=2.0, MIN_ACCEL=-3.5, INACTIVE_ACCEL=0.0) map to the raw limits
  # (2400 / 1300 / 2000) via the carcontroller encoding raw = accel * 200 + 2000.
  FLAGS = FLAG_MAZDA_GEN2 | FLAG_MAZDA_LONG

  def _accel_msg(self, accel: float):
    # 12-bit ACCEL_CMD packed per mazda.h decode: data[2] bit0=MSB, data[3]=mid 8, data[4] bits7..5=low 3
    raw = int(round(accel * 200 + 2000))
    dat = bytearray(8)
    dat[2] = (raw >> 11) & 0x01
    dat[3] = (raw >> 3) & 0xFF
    dat[4] = (raw & 0x07) << 5
    return libsafety_py.make_CANPacket(MAZDA_2019_ACC, MAZDA_CAM, bytes(dat))

  def test_gen2_acc_tx_allowed(self):
    # with FLAG_MAZDA_LONG the ACCEL_CMD is validated; use the inactive accel for the allowed-bus check
    self.assertTrue(self._tx(self._accel_msg(self.INACTIVE_ACCEL)))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(MAZDA_2019_ACC, MAZDA_MAIN, b"\x00" * 8)))


class TestMazdaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, byte0):
    return make_msg(0, 0x9E, dat=bytes([byte0]) + b"\x00" * 7)

  # 0x9E byte 0 high 3 bits == 6 (0xC0)
  def test_ignition_on(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    self.safety.ignition_can_hook(self._msg(0xC0))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0x20))
    self.assertFalse(self.safety.get_ignition_can())


class TestMazdaGen2Ignition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()

  def _msg(self, bus, byte5):
    dat = bytearray(8)
    dat[5] = byte5
    return make_msg(bus, 0x274, dat=bytes(dat))

  # 0x274 byte 5 bit 2 (0x4) set == ignition on; valid on bus 0 and bus 1
  def test_ignition_on_bus0(self):
    self.safety.ignition_can_hook(self._msg(0, 0x4))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off_bus0(self):
    self.safety.ignition_can_hook(self._msg(0, 0x4))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(0, 0x0))
    self.assertFalse(self.safety.get_ignition_can())

  def test_ignition_on_bus1(self):
    self.safety.ignition_can_hook(self._msg(1, 0x4))
    self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off_bus1(self):
    self.safety.ignition_can_hook(self._msg(1, 0x4))
    self.assertTrue(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._msg(1, 0x0))
    self.assertFalse(self.safety.get_ignition_can())

if __name__ == "__main__":
  unittest.main()
