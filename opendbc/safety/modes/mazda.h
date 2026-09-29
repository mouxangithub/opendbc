#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

#define MAZDA_2019_BRAKE          0x43FU
#define MAZDA_2019_GAS            0x202U
#define MAZDA_2019_CRUISE         0x44AU
#define MAZDA_2019_WHEEL_SPEEDS   0x215U
#define MAZDA_2019_SPEED          0x217U
#define MAZDA_2019_STEER_TORQUE   0x24BU
#define MAZDA_2019_CRZ_BTNS       0x09DU
#define MAZDA_2019_ACC            0x220U
#define MAZDA_TI_LKAS             0x249U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_AUX  1
#define MAZDA_CAM  2

// param flag masks
#define FLAG_MAZDA_GEN2 2U
#define FLAG_MAZDA_TORQUE_INTERCEPTOR 8U
// GEN2 openpilot longitudinal: when set, the TX hook validates ACCEL_CMD on MAZDA_2019_ACC
// against MAZDA_2019_LONG_LIMITS via longitudinal_accel_checks. When unset, ACCEL_CMD is
// passed through unchecked because it carries the stock cam ACC value (legacy mode).
#define FLAG_MAZDA_LONG 16U

static bool mazda_gen2 = false;
static bool mazda_torque_interceptor = false;
static bool mazda_longitudinal = false;

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if (!mazda_gen2 && ((int)msg->bus == MAZDA_MAIN)) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if (msg->addr == MAZDA_CRZ_CTRL) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      brake_pressed = (msg->data[0] & 0x10U);
    }
  }

  if (mazda_gen2) {
    if ((int)msg->bus == MAZDA_MAIN) {
      if (msg->addr == MAZDA_2019_BRAKE) {
        brake_pressed = (msg->data[5] & 0x4U) != 0U;
      }

      if (msg->addr == MAZDA_2019_CRUISE) {
        // CRZ_STATE is a 3-bit Motorola signal at byte 0 bits 6:4: 0=DISABLED, 1=READY (main on),
        // 2=ENABLED (cruise engaged), 4=GAS_OVERRIDE. Track acc_main_on off CRZ_STATE != 0 so it
        // produces real rising/falling edges when the driver toggles ACC main; otherwise MADS would
        // see a spurious rising edge on the very first CRUISE frame at boot, then never another.
        acc_main_on = (msg->data[0] & 0x70U) != 0U;
        bool cruise_engaged = (msg->data[0] & 0x20U) != 0U;
        bool pre_enable = (msg->data[0] & 0x40U) != 0U;
        pcm_cruise_check(cruise_engaged || pre_enable);
      }
    }

    if (((int)msg->bus == MAZDA_AUX) || (mazda_torque_interceptor && ((int)msg->bus == MAZDA_MAIN))) {
      if (msg->addr == MAZDA_2019_STEER_TORQUE) {
        int torque_driver_new = (int16_t)((msg->data[0] << 8) | msg->data[1]);
        update_sample(&torque_driver, torque_driver_new);
      }
    }

    if ((int)msg->bus == MAZDA_CAM) {
      if (msg->addr == MAZDA_2019_GAS) {
        gas_pressed = (msg->data[4] != 0U) || ((msg->data[5] & 0xC0U) != 0U);
      }

      if (msg->addr == MAZDA_2019_SPEED) {
        // sample speed: scale by 0.01 to get kph
        int speed = (msg->data[4] << 8) | msg->data[5];
        vehicle_moving = speed > 10; // moving when speed > 0.1 kph
        UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS);
      }

      if (msg->addr == MAZDA_2019_WHEEL_SPEEDS) {
        int speed = ((msg->data[0] << 8) | msg->data[1]) - 10000;
        vehicle_moving = speed > 10; // moving when speed > 0.1 kph
        UPDATE_VEHICLE_SPEED(speed * 0.01 * KPH_TO_MS);
      }
    }
  }
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  const TorqueSteeringLimits MAZDA_2019_STEERING_LIMITS = {
    .max_torque = 8000,
    .max_rate_up = 45,
    .max_rate_down = 80,
    .max_rt_delta = 1688,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 1400,
    .type = TorqueDriverLimited,
  };

  // GEN2 longitudinal limits in raw ACCEL_CMD units (carcontroller emits accel*200+2000).
  // +2.0 m/s^2 -> 2400, 0 m/s^2 -> 2000, -3.5 m/s^2 -> 1300. inactive_accel = 2000 matches the
  // sentinel the carcontroller writes when op_long=True but long_active=False, so the not-engaged
  // path passes longitudinal_accel_checks. (0 raw would be ~ -10 m/s^2 phantom braking.)
  const LongitudinalLimits MAZDA_2019_LONG_LIMITS = {
    .max_accel = 2400,
    .min_accel = 1300,
    .inactive_accel = 2000,
  };

  bool tx = true;
  // Check if msg is sent on the main BUS
  if (!mazda_gen2 && (msg->bus == (unsigned char)MAZDA_MAIN)) {
    // steer cmd checks
    if (msg->addr == MAZDA_LKAS) {
      int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

      if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_STEERING_LIMITS)) {
        tx = false;
      }
    }

    // cruise buttons check
    if (msg->addr == MAZDA_CRZ_BTNS) {
      // allow resume spamming while controls allowed, but
      // only allow cancel while controls not allowed
      bool cancel_cmd = (msg->data[0] == 0x1U);
      if (!controls_allowed && !cancel_cmd) {
        tx = false;
      }
    }
  }

  if (mazda_gen2 && (msg->bus == (unsigned char)MAZDA_AUX) && (msg->addr == MAZDA_TI_LKAS)) {
    int desired_torque = (int16_t)((msg->data[0] << 8) | msg->data[1]);
    if (steer_torque_cmd_checks(desired_torque, -1, MAZDA_2019_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // GEN2 longitudinal: ACCEL_CMD is a 12-bit Motorola-forward signal at DBC bit 16 length 12
  // (mazda_2019.dbc: 'SG_ ACCEL_CMD : 16|12@0+'). Empirically (verified against CANPacker output)
  // the bit layout in the wire bytes is:
  //   value bit 11 (MSB) = msg->data[2] bit 0
  //   value bits 10..3   = msg->data[3] bits 7..0
  //   value bits 2..0    = msg->data[4] bits 7..5
  // Only enforced when FLAG_MAZDA_LONG is set; otherwise ACCEL_CMD is the cam-pass-through value
  // and validating it would break stock-ACC operation (e.g. stock AEB requesting hard braking).
  if (mazda_longitudinal && (msg->bus == (unsigned char)MAZDA_CAM) && (msg->addr == MAZDA_2019_ACC)) {
    int accel_cmd = (int)((((uint16_t)(msg->data[2] & 0x01U)) << 11) |
                          (((uint16_t)msg->data[3]) << 3) |
                          (((uint16_t)(msg->data[4] >> 5)) & 0x07U));
    if (longitudinal_accel_checks(accel_cmd, MAZDA_2019_LONG_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

static bool mazda_fwd_hook(int bus_num, int addr) {
  bool block = false;

  if (mazda_gen2) {
    block = addr == MAZDA_TI_LKAS;
    if (bus_num == MAZDA_MAIN) {
      block = block || (addr == MAZDA_2019_ACC);
    }
  }

  return block;
}

static safety_config mazda_init(uint16_t param) {
  static const CanMsg MAZDA_TX_MSGS[] = {{MAZDA_LKAS, 0, 8, .check_relay = true}, {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false}, {MAZDA_LKAS_HUD, 0, 8, .check_relay = true}};
  static const CanMsg MAZDA_2019_TX_MSGS[] = {{MAZDA_TI_LKAS, 1, 8, .check_relay = true}, {MAZDA_2019_ACC, 2, 8, .check_relay = true}};

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static RxCheck mazda_2019_rx_checks[] = {
    {.msg = {{MAZDA_2019_BRAKE,        0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_GAS,          2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_CRUISE,       0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_SPEED,        2, 8, 30U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {MAZDA_2019_WHEEL_SPEEDS, 2, 8, 30U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},
    {.msg = {{MAZDA_2019_STEER_TORQUE, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static RxCheck mazda_2019_ti_rx_checks[] = {
    {.msg = {{MAZDA_2019_BRAKE,        0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_GAS,          2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_CRUISE,       0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_2019_SPEED,        2, 8, 30U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {MAZDA_2019_WHEEL_SPEEDS, 2, 8, 30U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},
    {.msg = {{MAZDA_2019_STEER_TORQUE, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},
             {MAZDA_2019_STEER_TORQUE, 1, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},
    {.msg = {{MAZDA_2019_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_gen2 = GET_FLAG(param, FLAG_MAZDA_GEN2);
  mazda_torque_interceptor = GET_FLAG(param, FLAG_MAZDA_TORQUE_INTERCEPTOR);
  mazda_longitudinal = GET_FLAG(param, FLAG_MAZDA_LONG);

  safety_config ret;
  if (mazda_gen2) {
    ret = mazda_torque_interceptor ? BUILD_SAFETY_CFG(mazda_2019_ti_rx_checks, MAZDA_2019_TX_MSGS) : \
                                    BUILD_SAFETY_CFG(mazda_2019_rx_checks, MAZDA_2019_TX_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
  }
  return ret;
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .fwd = mazda_fwd_hook,
};
