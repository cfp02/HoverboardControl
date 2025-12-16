#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// MAC is: AC:67:B2:53:86:28
// Another Mac: 24:0A:C4:1D:29:A0

/* ======================== Configuration ======================== */

// Hoverboard UART
static constexpr int      TX_HOVER   = TX;
static constexpr int      RX_HOVER   = RX;
static constexpr uint32_t HOVER_BAUD = 115200;

// Command protocol
static constexpr uint16_t START_FRAME        = 0xABCD;
static constexpr uint16_t SEND_PERIOD_MS     = 20;     // 50 Hz update rate
static constexpr uint32_t COMMAND_TIMEOUT_MS = 500;    // failsafe timeout
static constexpr int16_t  VOLT_FULL_UNITS    = 1000;   // firmware “100%”

/* ======================== Hoverboard Protocol ======================== */

#pragma pack(push,1)
struct SerialCommand {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
};

// Incoming Feedback structure from Hoverboard
struct SerialFeedback {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};
#pragma pack(pop)

namespace HB {
  HardwareSerial& port() { static HardwareSerial h(0); return h; }

  void begin() {
    port().begin(HOVER_BAUD, SERIAL_8N1, RX_HOVER, TX_HOVER);
  }

  inline uint16_t cs(const SerialCommand& c) {
    return static_cast<uint16_t>(c.start ^ c.steer ^ c.speed);
  }

  // Checksum calculation for received data
  inline uint16_t csRecv(const SerialFeedback& f) {
    return static_cast<uint16_t>(f.start ^ f.cmd1 ^ f.cmd2 ^ f.speedR_meas ^ f.speedL_meas
                            ^ f.batVoltage ^ f.boardTemp ^ f.cmdLed);
  }

  inline int16_t pctToUnits(float pct) {
    pct = fminf(fmaxf(pct, -100.0f), 100.0f);
    return static_cast<int16_t>(lroundf(pct * (VOLT_FULL_UNITS / 100.0f)));
  }

  void sendSpeedSteerPct(float speedPct, float steerPct) {
    SerialCommand c{START_FRAME, pctToUnits(steerPct), pctToUnits(speedPct)};
    c.checksum = cs(c);
    port().write(reinterpret_cast<const uint8_t*>(&c), sizeof(c));
  }

  // Check for incoming data
  bool readFeedback(SerialFeedback& data) {
    auto& p = port();
    if (p.available() < sizeof(SerialFeedback)) return false;

    // Basic sync: peek first byte, if not 0xABCD low byte (0xCD or 0xAB depending on endianness), skip
    // Simplify: Just read if size is there.
    // Note: 0xABCD is sent as Low Byte (0xCD) then High Byte (0xAB) usually.
    
    // Minimal implementation:
    if (p.peek() != 0xAB && p.peek() != 0xCD) {
        p.read(); // Discard misalignment
        return false;
    }

    SerialFeedback temp;
    p.readBytes((uint8_t*)&temp, sizeof(temp));
    
    if (temp.start == START_FRAME && temp.checksum == csRecv(temp)) {
        data = temp;
        return true;
    }
    return false;
  }
}

/* ======================== ESPNOW ======================== */

#pragma pack(push,1)
struct JoystickPacket {
  int16_t speed_pct;  // forward/backward
  int16_t steer_pct;  // left/right
  uint8_t flags;
  uint8_t seq;
  uint16_t crc;
};

// Packet to send back to controller
struct FeedbackPacket {
  SerialFeedback hb;
  uint16_t crc;
};
#pragma pack(pop)

// CRC helper
static uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
  while (len--) {
    crc ^= ((uint16_t)*data++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

static JoystickPacket lastJoy{};
static uint32_t tLastJoy = 0;
static bool peerAdded = false;

// Store controller address for reply
static uint8_t controllerMac[6];
static bool controllerKnown = false;

// Receive callback
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(JoystickPacket)) return;

  JoystickPacket p;
  memcpy(&p, data, sizeof(p));

  uint16_t crcCalc = crc16_ccitt((uint8_t*)&p, sizeof(p) - sizeof(p.crc));
  if (crcCalc != p.crc) return;

  // Capture sender MAC and register as peer if needed
  if (!controllerKnown || memcmp(controllerMac, mac, 6) != 0) {
      memcpy(controllerMac, mac, 6);
      controllerKnown = true;
      if (!esp_now_is_peer_exist(mac)) {
          esp_now_peer_info_t peer{};
          memcpy(peer.peer_addr, mac, 6);
          peer.channel = 0;
          peer.encrypt = false;
          esp_now_add_peer(&peer);
      }
  }

  lastJoy = p;
  tLastJoy = millis();
}

bool setupEspNow() {
  WiFi.mode(WIFI_STA); // Station mode only, no AP
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNowRecv);
  return true;
}

/* ======================== Main Loop ======================== */

void setup() {
  HB::begin();
  setupEspNow();
}

void loop() {
  const uint32_t now = millis();

  // Check and forward feedback
  SerialFeedback fb;
  if (HB::readFeedback(fb) && controllerKnown) {
      FeedbackPacket pkt;
      pkt.hb = fb;
      pkt.crc = crc16_ccitt((uint8_t*)&pkt.hb, sizeof(pkt.hb));
      esp_now_send(controllerMac, (uint8_t*)&pkt, sizeof(pkt));
  }

  // Check link timeout
  bool active = (now - tLastJoy) < COMMAND_TIMEOUT_MS;
  float spd = active ? lastJoy.speed_pct : 0.0f;
  float str = active ? lastJoy.steer_pct : 0.0f;

  static uint32_t tLastSend = 0;
  if (now - tLastSend >= SEND_PERIOD_MS) {
    HB::sendSpeedSteerPct(spd, str);
    tLastSend = now;
  }
}