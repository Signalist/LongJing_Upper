#include "Globals.h"
#include "CanTransport.h"

#if ENABLE_CAN
#include <driver/twai.h>

static bool canInstalled = false;
static CanTextReceiver<COMMAND_MAX_LENGTH> canRx;
static String canTxQueue[8];
static uint8_t canTxHead = 0, canTxCount = 0, canTxMessage = 0, canTxSequence = 0;
static size_t canTxOffset = 0;
static uint32_t canTxStarted = 0;
static uint32_t canRxErrors = 0, canTxDrops = 0;

static void dropCanTx() {
  if (!canTxCount) return;
  canTxQueue[canTxHead] = "";
  canTxHead = (canTxHead + 1) % 8;
  --canTxCount;
  canTxOffset = 0;
  canTxSequence = 0;
  ++canTxMessage;
  canTxStarted = millis();
}

void canSendText(const String& msg) {
  if (!canInstalled || !msg.length()) return;
  if (canTxCount == 8 || msg.length() > 2048) { ++canTxDrops; return; }
  if (!canTxCount) canTxStarted = millis();
  canTxQueue[(canTxHead + canTxCount) % 8] = msg;
  ++canTxCount;
}

void setupCAN() {
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      gpio_num_t(PIN_CAN_TX), gpio_num_t(PIN_CAN_RX), TWAI_MODE_NORMAL);
  general.tx_queue_len = 8;
  general.rx_queue_len = 32;
  general.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED |
                          TWAI_ALERT_RX_QUEUE_FULL;
#if CAN_BITRATE == 1000000
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
#elif CAN_BITRATE == 500000
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
#elif CAN_BITRATE == 250000
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
#else
#error Unsupported CAN_BITRATE
#endif
  twai_filter_config_t filter = {};
  filter.acceptance_code = uint32_t(CAN_COMMAND_ID) << 21;
  filter.acceptance_mask = 0x001fffff;
  filter.single_filter = true;
  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  if (err == ESP_OK) {
    err = twai_start();
    if (err != ESP_OK) twai_driver_uninstall();
  }
  canInstalled = err == ESP_OK;
  Serial.printf("CAN init: %s, bitrate=%lu, RX=0x%03X, TX=0x%03X\n",
                esp_err_to_name(err), (unsigned long)CAN_BITRATE, CAN_COMMAND_ID, CAN_REPLY_ID);
}

String getCanInfo() {
  twai_status_info_t status = {};
  String result = "CAN=";
  if (!canInstalled || twai_get_status_info(&status) != ESP_OK) return result + "disabled_or_failed";
  const char* state = status.state == TWAI_STATE_RUNNING ? "running" :
                      status.state == TWAI_STATE_BUS_OFF ? "bus_off" :
                      status.state == TWAI_STATE_RECOVERING ? "recovering" : "stopped";
  result += state;
  result += ", CAN_BITRATE=" + String(CAN_BITRATE);
  result += ", CAN_RX_ID=" + String(CAN_COMMAND_ID, HEX);
  result += ", CAN_TX_ID=" + String(CAN_REPLY_ID, HEX);
  result += ", CAN_RX_ERRORS=" + String(canRxErrors);
  result += ", CAN_TX_DROPS=" + String(canTxDrops);
  result += ", CAN_TX_FAILED=" + String(status.tx_failed_count);
  return result;
}

void handleCAN() {
  if (!canInstalled) return;
  uint32_t alerts = 0;
  twai_read_alerts(&alerts, 0);
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    ++canRxErrors; canRx.reset(); twai_clear_receive_queue();
  }
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) return;
  if (status.state == TWAI_STATE_BUS_OFF) {
    canRx.reset();
    while (canTxCount) { ++canTxDrops; dropCanTx(); }
    twai_clear_receive_queue();
    twai_initiate_recovery();
    return;
  }
  if (status.state == TWAI_STATE_RECOVERING) return;
  if (status.state == TWAI_STATE_STOPPED) {
    twai_start();
    return;
  }
  if (canRx.expired(millis(), CAN_FRAGMENT_TIMEOUT_MS)) {
    ++canRxErrors; canRx.reset();
  }
  // Bound work per FOC iteration, including commands from a saturated bus.
  twai_message_t frame = {};
  for (uint8_t n = 0; n < 4 && twai_receive(&frame, 0) == ESP_OK; ++n) {
    if (frame.extd || frame.rtr || frame.identifier != CAN_COMMAND_ID) continue;
    int result = canRx.feed(frame.data, frame.data_length_code, millis());
    if (result < 0) ++canRxErrors;
    if (result > 0) {
      handleCommand(String(canRx.text), 3);
      canRx.reset();
      break;
    }
  }
  if (!canTxCount) return;
  if (uint32_t(millis() - canTxStarted) >= CAN_TX_TIMEOUT_MS) {
    ++canTxDrops;
    twai_clear_transmit_queue();
    dropCanTx();
    return;
  }
  const String& msg = canTxQueue[canTxHead];
  size_t remaining = msg.length() - canTxOffset;
  size_t count = remaining > 6 ? 6 : remaining;
  frame = {};
  frame.identifier = CAN_REPLY_ID;
  frame.data_length_code = count + 2;
  frame.data[0] = canTxSequence | (canTxOffset == 0 ? 0x80 : 0) | (count == remaining ? 0x40 : 0);
  frame.data[1] = canTxMessage;
  memcpy(frame.data + 2, msg.c_str() + canTxOffset, count);
  if (twai_transmit(&frame, 0) == ESP_OK) {
    canTxOffset += count;
    canTxSequence = (canTxSequence + 1) & 0x3f;
    if (canTxOffset == msg.length()) dropCanTx();
  }
}
#else
void setupCAN() {}
void handleCAN() {}
void canSendText(const String&) {}
String getCanInfo() { return "CAN=disabled"; }
#endif
