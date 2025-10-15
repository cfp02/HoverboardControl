#include <Arduino.h>
#include <math.h>

/*
  ESP32 → Hoverboard FOC (USART) controller  — VLT_MODE (voltage control)
  - Sends a smooth throttle sweep using the standard hoverSerial packet:
      start(0xABCD), steer(int16), speed(int16), checksum = XOR(start ^ steer ^ speed)
  - In VLT_MODE, the "speed" field is a voltage/effort command (open-loop throttle).

  Wiring (USART2 example / left header on many boards):
    ESP32 TX0  (GPIO1 / TX) → Board RX2 (PA3)
    ESP32 RX0  (GPIO3 / RX) ← Board TX2 (PA2)  [feedback optional]
    GND ↔ GND

  NOTE: Using UART0 (TX/RX pins). Close the USB Serial Monitor while connected.
*/

/////////////////////// User Configuration ///////////////////////

// UART setup (UART0 is used; no USB monitor while connected)
static constexpr int      TX_HOVER   = TX;       // TX0 → board RX
static constexpr int      RX_HOVER   = RX;       // RX0 ← board TX (optional)
static constexpr uint32_t HOVER_BAUD = 115200;

// Protocol / timing
static constexpr uint16_t START_FRAME    = 0xABCD;
static constexpr uint16_t SEND_PERIOD_MS = 20;    // 50 Hz send rate
static constexpr uint32_t ARM_DELAY_MS   = 1000;  // send zero for a moment on boot

// Voltage command scaling (firmware-dependent — start conservatively)
static constexpr int16_t  VOLT_FULL_UNITS = 1000; // "100%" voltage command (tune if needed)

// Sweep shape and limits
static constexpr bool  SWEEP_SINE       = true;   // true: sine; false: triangle
static constexpr float SWEEP_AMPL_PCT   = 40.0f;  // ±percent of full scale (keep small)
static constexpr float SWEEP_FREQ_HZ    = 0.20f;  // sweep frequency (Hz)
static constexpr float THROTTLE_SLEW_PCT_PER_S = 60.0f; // rate limit for change (%/s)

// Steering (usually 0 for both wheels same effort)
static constexpr int16_t STEER_DEFAULT = 0;

/////////////////////// Protocol Structs ///////////////////////

#pragma pack(push, 1)
struct SerialCommand {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;     // in VLT_MODE: voltage/effort request
  uint16_t checksum;
};

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

/////////////////////// Comms (encapsulated) ///////////////////////

namespace HB {
  HardwareSerial& port() {
    static HardwareSerial hover(0);   // UART0 (TX/RX pins)
    return hover;
  }

  void begin() {
    // Dedicate UART0 to the hoverboard link (no Serial.begin here)
    port().begin(HOVER_BAUD, SERIAL_8N1, RX_HOVER, TX_HOVER);
  }

  inline uint16_t checksum(const SerialCommand& c) {
    return (uint16_t)(c.start ^ c.steer ^ c.speed);
  }

  void sendRaw(int16_t steer, int16_t value) {
    SerialCommand cmd;
    cmd.start = START_FRAME;
    cmd.steer = steer;
    cmd.speed = value;
    cmd.checksum = checksum(cmd);
    port().write(reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
  }

  // pct in [-100..100] → firmware units (±VOLT_FULL_UNITS), with clamp
  inline int16_t throttlePctToUnits(float pct) {
    if (pct >  100.0f) pct =  100.0f;
    if (pct < -100.0f) pct = -100.0f;
    float units = pct * (VOLT_FULL_UNITS / 100.0f);
    if (units >  (float)VOLT_FULL_UNITS) units =  (float)VOLT_FULL_UNITS;
    if (units < -(float)VOLT_FULL_UNITS) units = -(float)VOLT_FULL_UNITS;
    return (int16_t)lroundf(units);
  }

  // Convenience: send percentage throttle with optional steering
  void sendThrottlePct(float pct, int16_t steer = STEER_DEFAULT) {
    sendRaw(steer, throttlePctToUnits(pct));
  }

  // Optional: parse feedback (not required for sweep; left here for completeness)
  bool pollFeedback(SerialFeedback& out) {
    static uint8_t  idx = 0;
    static uint16_t startWin = 0;
    static uint8_t* p = nullptr;
    static SerialFeedback fbNew;

    while (port().available()) {
      uint8_t b = port().read();
      startWin = (uint16_t)((startWin << 8) | b);

      if (idx == 0 && startWin == START_FRAME) {
        p = (uint8_t*)&fbNew;
        *p++ = (uint8_t)(START_FRAME & 0xFF);
        *p++ = (uint8_t)((START_FRAME >> 8) & 0xFF);
        idx = 2;
        continue;
      }
      if (idx >= 2 && idx < sizeof(SerialFeedback)) {
        *p++ = b;
        idx++;
        if (idx == sizeof(SerialFeedback)) {
          uint16_t cs = fbNew.start ^ fbNew.cmd1 ^ fbNew.cmd2 ^ fbNew.speedR_meas ^
                        fbNew.speedL_meas ^ fbNew.batVoltage ^ fbNew.boardTemp ^ fbNew.cmdLed;
          bool ok = (fbNew.start == START_FRAME && cs == fbNew.checksum);
          idx = 0;
          if (ok) { out = fbNew; return true; }
          return false;
        }
      }
    }
    return false;
  }
} // namespace HB

/////////////////////// Control Helpers ///////////////////////

// Rate-limit a percent value (for smooth throttle changes)
float slewPercent(float current, float target, float dt_s) {
  const float maxDelta = THROTTLE_SLEW_PCT_PER_S * dt_s;
  float d = target - current;
  if (d >  maxDelta) d =  maxDelta;
  if (d < -maxDelta) d = -maxDelta;
  return current + d;
}

// Triangle wave in [-ampl, +ampl]
float triangleWave(float t, float freq_hz, float ampl) {
  const float period = 1.0f / (freq_hz > 0 ? freq_hz : 1.0f);
  float x = fmodf(t, period) / period; // [0,1)
  float tri = 4.0f * fabsf(x - 0.5f) - 1.0f; // [-1,1]
  return tri * ampl;
}

/////////////////////// App ///////////////////////

uint32_t tBoot = 0;
uint32_t tLastSend = 0;
float    throttlePct = 0.0f; // what we actually send (slew-limited)

void setup() {
  HB::begin();
  tBoot = millis();
  tLastSend = tBoot;
}

void loop() {
  const uint32_t now = millis();

  // Arming delay: hold zero for a short time after boot
  if (now - tBoot < ARM_DELAY_MS) {
    if (now - tLastSend >= SEND_PERIOD_MS) {
      HB::sendThrottlePct(0.0f);
      tLastSend = now;
    }
    return;
  }

  // Generate target throttle percent (smooth sweep)
  const float t = 0.001f * (now - tBoot); // seconds since boot
  float targetPct;
  if (SWEEP_SINE) {
    targetPct = SWEEP_AMPL_PCT * sinf(2.0f * PI * SWEEP_FREQ_HZ * t);
  } else {
    targetPct = triangleWave(t, SWEEP_FREQ_HZ, SWEEP_AMPL_PCT);
  }

  // Slew-limit the throttle
  const float dt = (now - tLastSend) * 0.001f;
  if (dt > 0.0f) {
    throttlePct = slewPercent(throttlePct, targetPct, dt);
  }

  // Send at 50 Hz
  if (now - tLastSend >= SEND_PERIOD_MS) {
    HB::sendThrottlePct(throttlePct, STEER_DEFAULT);
    tLastSend = now;
  }

  // (Optional) Read feedback if RX is wired:
  // SerialFeedback fb;
  // if (HB::pollFeedback(fb)) {
  //   // Inspect fb.speedL_meas / fb.speedR_meas / fb.batVoltage etc.
  // }
}