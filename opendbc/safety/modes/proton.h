#define PROTON_GAS_SENSOR       0x201
#define PROTON_STEERING_TORQUE  0x150
#define PROTON_BRAKE            0x123
#define PROTON_ENGINE           0x082
#define PROTON_GAS_PEDAL        0x084
#define PROTON_PCM_BUTTONS      0x1A3
#define PROTON_GAS_COMMAND      0x200
#define PROTON_TORQUE_COMMAND   0x202
#define PROTON_ACC_CMD          0x1A1
#define PROTON_ADAS_LKAS        0x1B0

static void proton_rx_hook(const CANPacket_t *msg) {
  int bus = msg->bus;
  int addr = msg->addr;
  // proton is never at standstill
  vehicle_moving = true;

  if (addr == 0x201U) {
    controls_allowed = true;
  }
}

static bool proton_tx_hook(const CANPacket_t *msg) {
  int bus = msg->bus;
  int addr = msg->addr;
  int len = GET_LEN(msg);
  bool tx = true;

  if (!controls_allowed && (addr != 0x1)) {
    tx = false;
  }

  // Allow going into CAN flashing mode for base & knee even if controls are not allowed
  bool flash_msg = ((addr == 0x250) || (addr == 0x350)) && (len == 8);
  if (!controls_allowed && (GET_BYTES(msg, 0, 4) == 0xdeadfaceU) && (GET_BYTES(msg, 4, 4) == 0x0ab00b1eU) && flash_msg) {
    tx = true;
  }

  return tx;
}

static safety_config proton_init(uint16_t param) {
  const CanMsg PROTON_TX_MSGS[] = {{0x250, 0, 8}, {0x250, 0, 6}, {0x251, 0, 5},  // proton
                               {0x350, 0, 8}, {0x350, 0, 6}, {0x351, 0, 5},  // knee
                               {0x1, 0, 8}}; // CAN flasher
  static RxCheck proton_rx_checks[] = {
    {.msg = {{0x201, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
  };
  UNUSED(param);
  return BUILD_SAFETY_CFG(proton_rx_checks, PROTON_TX_MSGS);
}

const safety_hooks proton_hooks = {
  .init = proton_init,
  .rx = proton_rx_hook,
  .tx = proton_tx_hook,
  .fwd = default_fwd_hook,
};
