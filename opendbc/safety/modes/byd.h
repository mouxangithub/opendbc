#pragma once

#include "opendbc/safety/declarations.h"

#define BYD_CANADDR_IPB               0x1F0
#define BYD_CANADDR_ACC_MPC_STATE     0x316
#define BYD_CANADDR_ACC_EPS_STATE     0x318
#define BYD_CANADDR_ACC_HUD_ADAS      0x32D
#define BYD_CANADDR_ACC_CMD           0x32E
#define BYD_CANADDR_PCM_BUTTONS       0x3B0
#define BYD_CANADDR_DRIVE_STATE       0x242
#define BYD_CANADDR_PEDAL             0x342
#define BYD_CANADDR_CARSPEED          0x121

// ATTO3 (byd_general.dbc) addresses
#define BYD_CANADDR_STEERING_TORQUE       0x1FC  // MAIN_TORQUE from EPS (NOT driver input)
#define BYD_CANADDR_STEER_MODULE_2        0x11F  // STEER_ANGLE_2 + DRIVER_EPS_TORQUE
#define BYD_CANADDR_STEERING_MODULE_ADAS  0x1E2  // LKAS steering command
#define BYD_CANADDR_WHEEL_SPEED           0x122  // per-wheel speeds

#define BYD_CANBUS_ESC  0
#define BYD_CANBUS_MRR  1
#define BYD_CANBUS_MPC  2

// BYD's checksum key: nibble sums of every byte but the last, folded with a fixed key.
#define BYD_CHECKSUM_KEY 0xAFU
// How long the camera's copy stays blocked after openpilot sends one. Longer than a few
// missed frames at 50Hz, short enough to hand back promptly on disengage.
#define BYD_TAKEOVER_TIMEOUT 150000U  // us

static bool byd_eps_cruiseactivated = false;
// Tracked separately: the HUD carries no safety checks, so a HUD frame must never be
// able to keep the camera's *steering* command blocked.
static bool byd_steering = false;
static uint32_t byd_last_steer_tx = 0U;
static bool byd_hud = false;
static uint32_t byd_last_hud_tx = 0U;

typedef enum {
  HAN_TANG_DMEV,
  TANG_DMI,
  SONG_PLUS_DMI,
  QIN_PLUS_DMI,
  ATTO3_GENERAL
} BydPlatform;
static BydPlatform byd_platform;

// BYD's checksum: nibble sums of every byte but the last, folded with a fixed key.
static uint32_t byd_compute_checksum(const CANPacket_t *msg) {
  const uint8_t key = (uint8_t)BYD_CHECKSUM_KEY;
  int len = GET_LEN(msg);

  uint8_t high_sum = 0U;
  uint8_t low_sum = 0U;
  for (int i = 0; i < (len - 1); i++) {
    high_sum += (uint8_t)(msg->data[i] >> 4);
    low_sum += (uint8_t)(msg->data[i] & 0xFU);
  }

  // the remainder is taken before the key nibbles are folded in
  const uint8_t remainder = (uint8_t)(low_sum >> 4);
  high_sum += (uint8_t)(key & 0xFU);
  low_sum += (uint8_t)(key >> 4);

  const uint8_t high_part = (uint8_t)((0x9U - high_sum) & 0xFU);
  const uint8_t low_part = (uint8_t)((0x9U - low_sum) & 0xFU);
  const uint8_t folded = (uint8_t)((high_part + 5U) - remainder);

  return (uint32_t)((uint8_t)(((uint32_t)folded * 16U) + (uint32_t)low_part));
}

static uint32_t byd_get_checksum(const CANPacket_t *msg) {
  return (uint32_t)msg->data[GET_LEN(msg) - 1U];
}

static uint8_t byd_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0U;
  if ((msg->addr == BYD_CANADDR_ACC_HUD_ADAS) || (msg->addr == BYD_CANADDR_ACC_CMD)) {
    // COUNTER : 48|4@1+ (big-endian, low nibble of byte 6)
    cnt = msg->data[6] & 0xFU;
  } else {
    // COUNTER : 55|4@0+ (little-endian, high nibble of byte 6)
    cnt = (uint8_t)(msg->data[6] >> 4);
  }
  return cnt;
}

static bool byd_takeover_expired(bool owned, uint32_t last_tx) {
  return owned && (safety_get_ts_elapsed(microsecond_timer_get(), last_tx) >= BYD_TAKEOVER_TIMEOUT);
}

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = to_push->bus;
  int addr = to_push->addr;

  if (byd_platform == ATTO3_GENERAL) {
    // ATTO3 export model: angle-control on byd_general.dbc, camera on bus 2 (MPC)
    if (bus == BYD_CANBUS_ESC) {
      if (addr == BYD_CANADDR_PEDAL) {
        gas_pressed = (to_push->data[0] != 0U);
        brake_pressed = (to_push->data[1] != 0U);
      } else if (addr == BYD_CANADDR_WHEEL_SPEED) {
        // WHEELSPEED_BR (bits 48-63): byte 7 is a constant status byte (0x41),
        // NOT the high byte of the wheel speed. DBC wrongly declares it as 16-bit.
        // Sum the three trustworthy wheels.
        uint16_t speed_fl = (uint16_t)((to_push->data[1] << 8) | to_push->data[0]);
        uint16_t speed_fr = (uint16_t)((to_push->data[3] << 8) | to_push->data[2]);
        uint16_t speed_rl = (uint16_t)((to_push->data[5] << 8) | to_push->data[4]);
        vehicle_moving = (speed_fl | speed_fr | speed_rl) != 0U;
        // DBC factor 0.1 gives km/h, but BYD ATTO3 India raw values read ~32.5% high
        // vs odometer at steady-state. Correction: 40/53. Verify with GPS if re-calibrating.
        UPDATE_VEHICLE_SPEED(((speed_fl + speed_fr + speed_rl) / 3.0) * 0.1 * (40.0 / 53.0) * KPH_TO_MS);
      } else if (addr == BYD_CANADDR_STEER_MODULE_2) {
        // STEER_MODULE_2 carries the column torque and the measured wheel angle.
        // DRIVER_EPS_TORQUE is the column sensor (unsigned magnitude); MAIN_TORQUE in
        // STEERING_TORQUE is EPS motor output and must not be used for driver override.
        int angle_meas_new = (int)GET_BYTES(to_push, 0, 2);      // STEER_ANGLE_2 : 0|16@1-
        angle_meas_new = to_signed(angle_meas_new, 16);
        update_sample(&angle_meas, angle_meas_new);

        int torque_driver_new = (int)to_push->data[2];           // DRIVER_EPS_TORQUE : 16|8@1+
        update_sample(&torque_driver, torque_driver_new);
      } else if (addr == BYD_CANADDR_STEERING_TORQUE) {
        // EPS motor torque (used for diagnostic logging only).
        int torque_motor_new = (int)GET_BYTES(to_push, 1, 2);    // MAIN_TORQUE : 8|12@1-
        if (torque_motor_new >= 2048) torque_motor_new -= 4096;
        update_sample(&torque_meas, torque_motor_new);
      }
    } else if (bus == BYD_CANBUS_MPC) {
      if (addr == BYD_CANADDR_ACC_CMD) {
        bool acc_on_1 = GET_BIT(to_push, 9U) != 0U;
        bool acc_on_2 = GET_BIT(to_push, 17U) != 0U;
        bool cmd_req_active_low = GET_BIT(to_push, 36U) != 0U;
        pcm_cruise_check(acc_on_1 && acc_on_2 && !cmd_req_active_low);
      } else if (addr == BYD_CANADDR_ACC_HUD_ADAS) {
        // ACC_HUD_ADAS carries the dashboard ACC main switch state on ATTO3
        bool acc_hud_on1 = GET_BIT(to_push, 22U) != 0U;
        bool acc_hud_on2 = GET_BIT(to_push, 20U) != 0U;
        acc_main_on = acc_hud_on1 || acc_hud_on2;
      }
    }
  } else {
    // Existing torque-controlled platforms (HAN/TANG/SONG/QIN)
    if (bus == BYD_CANBUS_ESC) {
      if (addr == BYD_CANADDR_PEDAL) {
        gas_pressed = (to_push->data[0] != 0U);
        brake_pressed = (to_push->data[1] != 0U);
      } else if (addr == BYD_CANADDR_CARSPEED) {
        int speed_raw = (((to_push->data[1] & 0x0FU) << 8) | to_push->data[0]);
        vehicle_moving = (speed_raw != 0);
      } else if (addr == BYD_CANADDR_ACC_EPS_STATE) {
        byd_eps_cruiseactivated = GET_BIT(to_push, 1U) != 0U; // CruiseActivated
        int torque_motor = (((to_push->data[2] & 0x0FU) << 8) | to_push->data[1]); // MainTorque
        if ( torque_motor >= 2048 )
          torque_motor -= 4096;
        update_sample(&torque_meas, torque_motor);
      }
      else {
        //empty
      }
    } else if (bus == BYD_CANBUS_MPC) {
      if (addr == BYD_CANADDR_ACC_HUD_ADAS) {
        unsigned int accstate = ((to_push->data[2] >> 3) & 0x07U);
        // ACC main on when system is active/standby (2=acc_on, 3=acc_active, 5=user force accel)
        acc_main_on = (accstate == 2U) || (accstate == 3U) || (accstate == 5U);
        bool cruise_engaged = (accstate == 3U) || (accstate == 5U); // 3=acc_active, 5=user force accel
        pcm_cruise_check(cruise_engaged);
      }
    }
    else {
      //empty
    }
  }
}


static bool byd_tx_hook(const CANPacket_t *to_send) {
  const TorqueSteeringLimits HAN_DMEV_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_torque_error = 80,
    .max_rt_delta = 243,
    .type = TorqueMotorLimited,
  };
  const TorqueSteeringLimits TANG_DMI_STEERING_LIMITS = { //values to be check
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_rt_delta = 243,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };
  const TorqueSteeringLimits SONG_STEERING_LIMITS = { //values to be check
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_rt_delta = 243,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };
  const TorqueSteeringLimits QIN_STEERING_LIMITS = { //values to be check
    .max_torque = 300,
    .max_rate_up = 17,
    .max_rate_down = 17,
    .max_rt_delta = 243,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };
  static const AngleSteeringLimits BYD_ATTO3_GENERAL_STEERING_LIMITS = {
    .max_angle = 900,  // 90 deg * 10 (DBC factor 0.1 deg)
    .angle_deg_to_can = 10.0,
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.4}
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {2.5, 1.5, 0.6}
    },
    .frequency = 50U,
  };

  bool tx = true;
  int bus = to_send->bus;
  int addr = to_send->addr;

  if (byd_platform == ATTO3_GENERAL) {
    if ((bus == BYD_CANBUS_ESC) && (addr == BYD_CANADDR_STEERING_MODULE_ADAS)) {
      int desired_angle = (int)GET_BYTES(to_send, 3, 2);  // STEER_ANGLE : 24|16@1-
      desired_angle = to_signed(desired_angle, 16);
      bool steer_control_enabled = GET_BIT(to_send, 21U) != 0U; // STEER_REQ

      if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, BYD_ATTO3_GENERAL_STEERING_LIMITS)) {
        tx = false;
      }
    }

    // Track which LKAS messages openpilot currently owns, so the fwd_hook can hand
    // control back to the camera if openpilot stops transmitting.
    if (tx && (bus == BYD_CANBUS_ESC)) {
      if (addr == BYD_CANADDR_STEERING_MODULE_ADAS) {
        byd_steering = true;
        byd_last_steer_tx = microsecond_timer_get();
      } else if (addr == BYD_CANADDR_ACC_MPC_STATE) {
        byd_hud = true;
        byd_last_hud_tx = microsecond_timer_get();
      }
    }
  } else {
    if ((bus == BYD_CANBUS_ESC) && (addr == BYD_CANADDR_ACC_MPC_STATE)) {
      int desired_torque = ((to_send->data[3] & 0x07U) << 8U) | to_send->data[2];
      bool steer_req = GET_BIT(to_send, 28U) && byd_eps_cruiseactivated; //LKAS_Active
      if ( desired_torque >= 1024 )
        desired_torque -= 2048;
      const TorqueSteeringLimits limits = (byd_platform == HAN_TANG_DMEV) ? HAN_DMEV_STEERING_LIMITS :
                                          (byd_platform == TANG_DMI) ? TANG_DMI_STEERING_LIMITS :
                                          (byd_platform == SONG_PLUS_DMI) ? SONG_STEERING_LIMITS : QIN_STEERING_LIMITS;

      if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
        tx = false;
      }
    }
  }

  return tx;
}

static bool byd_fwd_hook(int bus, int addr) {
  bool block_msg = false;

  if (byd_platform == ATTO3_GENERAL) {
    // ATTO3: camera is on bus 2 (MPC). Block its LKAS messages so openpilot can substitute.
    // Hand the camera back if openpilot hasn't steered within the takeover window —
    // that way the stock LKAS resumes control if we ever freeze or exit.
    const bool is_steer = (addr == BYD_CANADDR_STEERING_MODULE_ADAS);
    const bool is_hud = (addr == BYD_CANADDR_ACC_MPC_STATE);
    if (bus == BYD_CANBUS_MPC) {
      if (is_steer) {
        if (byd_takeover_expired(byd_steering, byd_last_steer_tx)) {
          byd_steering = false;
        }
        block_msg = byd_steering;
      } else if (is_hud) {
        if (byd_takeover_expired(byd_hud, byd_last_hud_tx)) {
          byd_hud = false;
        }
        block_msg = byd_hud;
      }
    }
  } else {
    const bool is_lkas = ((addr == BYD_CANADDR_ACC_MPC_STATE) || (addr == BYD_CANADDR_ACC_CMD));
    const bool is_eps = (addr == BYD_CANADDR_ACC_EPS_STATE);

    if ( ((bus == BYD_CANBUS_ESC) && is_eps) || ((bus == BYD_CANBUS_MPC) && is_lkas) ) {
      block_msg = true;
    }
  }

  return block_msg;
}

static safety_config byd_init(uint16_t param) {

  const uint32_t FLAG_HAN_TANG_DMEV = 0x1U;
  const uint32_t FLAG_TANG_DMI = 0x2U;
  const uint32_t FLAG_SONG_PLUS_DMI = 0x4U;
  const uint32_t FLAG_QIN_PLUS_DMI = 0x8U;
  const uint32_t FLAG_YUAN_PLUS_DMI_ATTO3 = 0x10U;
  const uint32_t FLAG_ATTO3_GENERAL = 0x20U;

  static const CanMsg BYD_HAN_DMEV_TX_MSGS[] = {
    {BYD_CANADDR_ACC_CMD,         BYD_CANBUS_ESC, 8, .check_relay = false, .disable_static_blocking = false},
    {BYD_CANADDR_ACC_MPC_STATE,   BYD_CANBUS_ESC, 8, .check_relay = true, .disable_static_blocking = false},
    {BYD_CANADDR_ACC_EPS_STATE,   BYD_CANBUS_MPC, 8, .check_relay = false, .disable_static_blocking = false},
  };

  static const CanMsg BYD_ATTO3_GENERAL_TX_MSGS[] = {
    {BYD_CANADDR_STEERING_MODULE_ADAS, BYD_CANBUS_ESC, 8, .check_relay = true, .disable_static_blocking = true},
    {BYD_CANADDR_ACC_MPC_STATE,        BYD_CANBUS_ESC, 8, .check_relay = true, .disable_static_blocking = true}, // 0x316 is LKAS_HUD_ADAS on ATTO3
  };

  // ignore_checksum / ignore_counter / ignore_quality_flag are NOT set wherever the
  // safety_hooks `get_checksum` / `compute_checksum` / `get_counter` callbacks are
  // registered — that way the safety framework validates every received frame. Frames
  // that don't have a checksum/counter in the DBC are explicitly opt-out below.
  static RxCheck byd_han_dmev_rx_checks[] = {
    {.msg = {{BYD_CANADDR_ACC_EPS_STATE,    BYD_CANBUS_ESC, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_CARSPEED,         BYD_CANBUS_ESC, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_DRIVE_STATE,      BYD_CANBUS_ESC, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,     BYD_CANBUS_MPC, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  static RxCheck byd_atto3_general_rx_checks[] = {
    {.msg = {{BYD_CANADDR_STEERING_TORQUE,  BYD_CANBUS_ESC, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_STEER_MODULE_2,   BYD_CANBUS_ESC, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_WHEEL_SPEED,      BYD_CANBUS_ESC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_PEDAL,            BYD_CANBUS_ESC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_DRIVE_STATE,      BYD_CANBUS_ESC, 8,  50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_CMD,          BYD_CANBUS_MPC, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{BYD_CANADDR_ACC_HUD_ADAS,     BYD_CANBUS_MPC, 8,  50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  safety_config ret;

  bool use_han_dm = GET_FLAG(param, FLAG_HAN_TANG_DMEV);
  bool use_tang_dmi = GET_FLAG(param, FLAG_TANG_DMI);
  bool use_song = GET_FLAG(param, FLAG_SONG_PLUS_DMI);
  bool use_qin = GET_FLAG(param, FLAG_QIN_PLUS_DMI);
  bool use_yuan = GET_FLAG(param, FLAG_YUAN_PLUS_DMI_ATTO3);
  bool use_atto3_general = GET_FLAG(param, FLAG_ATTO3_GENERAL);

  if(use_han_dm) {
    byd_platform = HAN_TANG_DMEV;
    ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  } else if (use_tang_dmi || use_song || use_qin) {
    byd_platform = TANG_DMI;
    ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  } else if (use_yuan || use_atto3_general) {
    // Yuan Plus uses the same byd_general.dbc angle-control port as the export ATTO3.
    // Keep the safety flag distinct (YUAN_PLUS_DMI_ATTO3 vs ATTO3_GENERAL) but share logic.
    byd_platform = ATTO3_GENERAL;
    ret = BUILD_SAFETY_CFG(byd_atto3_general_rx_checks, BYD_ATTO3_GENERAL_TX_MSGS);
  } else {
    //should not reach here
    byd_platform = HAN_TANG_DMEV;
    ret = BUILD_SAFETY_CFG(byd_han_dmev_rx_checks, BYD_HAN_DMEV_TX_MSGS);
  }

  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
  .get_checksum = byd_get_checksum,
  .compute_checksum = byd_compute_checksum,
  .get_counter = byd_get_counter,
};
