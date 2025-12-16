#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// MAC is: AC:67:B2:53:86:28
// Another Mac: 24:0A:C4:1D:29:A0

/* ======================== Configuration ======================== */

// Hoverboard UART
static constexpr int      TX_HOVER   = 16; // TX;
static constexpr int      RX_HOVER   = 17; //RX;
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
  // CHANGED: Use Serial2 for Hoverboard (leaves Serial0 free for USB debug)
  HardwareSerial& port() { static HardwareSerial h(2); return h; }

  void begin() {
    // Start the Hoverboard connection on the new pins
    port().begin(HOVER_BAUD, SERIAL_8N1, RX_HOVER, TX_HOVER);
  }

  inline uint16_t cs(const SerialCommand& c) {
    return static_cast<uint16_t>(c.start ^ c.steer ^ c.speed);
  }

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

  bool readFeedback(SerialFeedback& data) {
    auto& p = port();
    
    // DEBUG: Check if we are receiving ANY bytes at all
    if (p.available() > 0 && p.available() < sizeof(SerialFeedback)) {
       // Optional: Uncomment to see if bytes are trickling in
       // Serial.print("."); 
    }

    if (p.available() < sizeof(SerialFeedback)) return false;

    // Peek for sync byte (Low byte of 0xABCD is 0xCD or 0xAB depending on sender endianness)
    // Firmware sends: (uint16_t)start = 0xABCD. 
    // Usually sent as 0xCD then 0xAB (Little Endian).
    if (p.peek() != 0xAB && p.peek() != 0xCD) {
        // DEBUG: Found garbage byte
        byte trash = p.read();
        Serial.print("[Trash] 0x");
        Serial.println(trash, HEX);
        return false;
    }

    SerialFeedback temp;
    p.readBytes((uint8_t*)&temp, sizeof(temp));
    
    if (temp.start == START_FRAME && temp.checksum == csRecv(temp)) {
        data = temp;
        // DEBUG: SUCCESS!
        Serial.print("RX Valid! Volts: ");
        Serial.print(temp.batVoltage);
        Serial.print(" | L: ");
        Serial.print(temp.speedL_meas);
        Serial.print(" | R: ");
        Serial.println(temp.speedR_meas);
        return true;
    } else {
        // DEBUG: Checksum Failed
        Serial.print("CRC FAIL! Exp: ");
        Serial.print(csRecv(temp));
        Serial.print(" Got: ");
        Serial.println(temp.checksum);
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
static uint8_t controllerMac[6];
static bool controllerKnown = false;

void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(JoystickPacket)) return;
  JoystickPacket p;
  memcpy(&p, data, sizeof(p));
  uint16_t crcCalc = crc16_ccitt((uint8_t*)&p, sizeof(p) - sizeof(p.crc));
  if (crcCalc != p.crc) return;

  if (!controllerKnown || memcmp(controllerMac, mac, 6) != 0) {
      memcpy(controllerMac, mac, 6);
      controllerKnown = true;
      Serial.print("New Controller Found: ");
      Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      
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
  WiFi.mode(WIFI_STA); 
  if (esp_now_init() != ESP_OK) {
      Serial.println("ESPNow Init Failed");
      return false;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  Serial.println("ESPNow Ready");
  return true;
}

/* ======================== Main Loop ======================== */

void setup() {
  // Start USB Debugging
  Serial.begin(115200);
  Serial.println("Booting...");

  // Start Hoverboard on Serial2
  HB::begin();
  
  setupEspNow();
}

void loop() {
  const uint32_t now = millis();

  // Check Feedback
  SerialFeedback fb;
  if (HB::readFeedback(fb) && controllerKnown) {
      FeedbackPacket pkt;
      pkt.hb = fb;
      pkt.crc = crc16_ccitt((uint8_t*)&pkt.hb, sizeof(pkt.hb));
      esp_now_send(controllerMac, (uint8_t*)&pkt, sizeof(pkt));
  }

  bool active = (now - tLastJoy) < COMMAND_TIMEOUT_MS;
  float spd = active ? lastJoy.speed_pct : 0.0f;
  float str = active ? lastJoy.steer_pct : 0.0f;

  static uint32_t tLastSend = 0;
  if (now - tLastSend >= SEND_PERIOD_MS) {
    HB::sendSpeedSteerPct(spd, str);
    tLastSend = now;
  }
  
  // Debug output if no packet received for a while?
  static uint32_t tLastDebug = 0;
  if (now - tLastDebug > 2000) {
      if (!controllerKnown) Serial.println("Waiting for Controller...");
      tLastDebug = now;
  }
}