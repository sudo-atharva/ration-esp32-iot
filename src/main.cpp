// E-Ration node: fingerprint match -> distributor approves on website -> weigh out from 3 hoppers.
// Board: ESP32. Libraries (Arduino Library Manager): Adafruit Fingerprint Sensor Library,
// HX711 (Rob Tillaart), ESP32Servo, ArduinoJson (v7).
// Server protocol: see api/node.php.
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Fingerprint.h>
#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <Arduino.h>

// ---- Config ----
const char* WIFI_SSID = "Hotspot";
const char* WIFI_PASS = "12345678";  // Wi-Fi credentials for the ESP32 to connect to the distributor's hotspot
const char* API_URL   = "http://10.116.145.217:8000/api/node.php";  // IP of the PC running start-demo.bat
const char* NODE_KEY  = "5bccdc8d88e34a3c9ba8efc9";               // tbl_node.node_key for this shop (PDS 1201)

// ---- Pins ----
const int FP_RX = 17, FP_TX = 16;          // sensor TX -> GPIO17, sensor RX -> GPIO16 (AS608, 57600 baud)
const int HX_DT = 4, HX_SCK = 5;
const int HOPPER_PINS[3] = {32, 33, 25};   // L293D inputs (IN1/IN2/IN3), other side of each motor to GND, EN pins to 5V: 0 Wheat, 1 Rice, 2 Sugar. HIGH=run, LOW=stop.
const char* HOPPER_NAME[3] = {"Wheat", "Rice", "Sugar"};
const int LED = 2;
const int LCD_SDA = 21, LCD_SCL = 22;      // I2C 16x2 LCD (ESP32 default I2C pins, address 0x27)
const int BTN_NEXT = 13, BTN_STOP = 14;    // latching switches to GND; NEXT = dispense next item, STOP = end/skip

// ---- Calibration (tune on the real machine) ----
#define CALIBRATE 0                          // 1 = print raw scale readings to find SCALE_FACTOR
float SCALE_FACTOR = 420.0;                  // raw units per gram: put a known weight on, factor = raw / grams
const float STOP_EARLY_G[3] = {25, 25, 15};  // grams still falling after the motor stops (auger coast), per hopper
const unsigned long ITEM_TIMEOUT_MS = 60000; // hopper empty/jammed: stop and bill only what was given
const unsigned long POLL_MS = 1000;

HardwareSerial fpSerial(2);
Adafruit_Fingerprint finger(&fpSerial);
HX711 scale;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void blink(int n) {
  for (int i = 0; i < n; i++) { digitalWrite(LED, HIGH); delay(150); digitalWrite(LED, LOW); delay(150); }
}

// Two LCD lines (each truncated to 16 chars by the display).
void lcd2(const char* a, const char* b) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(a);
  lcd.setCursor(0, 1); lcd.print(b);
}

// Buttons are latching switches: each press flips the pin and it stays there, so a press = any change of state.
int lastNext, lastStop;
void syncButtons() { lastNext = digitalRead(BTN_NEXT); lastStop = digitalRead(BTN_STOP); }  // forget flips made while not asked

bool flipped(int pin, int& last) {
  int now = digitalRead(pin);
  if (now == last) return false;
  delay(30);  // debounce
  if (digitalRead(pin) != now) return false;
  last = now;
  return true;
}

// Block until NEXT or STOP is pressed. Returns true for NEXT, false for STOP.
bool waitButton() {
  syncButtons();
  for (;;) {
    if (flipped(BTN_NEXT, lastNext)) return true;
    if (flipped(BTN_STOP, lastStop)) return false;
    delay(10);
  }
}

// POST {key, ...req} to the server, parse JSON reply into res.
bool api(JsonDocument& req, JsonDocument& res) {
  if (WiFi.status() != WL_CONNECTED) return false;
  req["key"] = NODE_KEY;
  String body;
  serializeJson(req, body);
  HTTPClient http;
  http.begin(API_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);
  int code = http.POST(body);
  bool ok = code > 0 && !deserializeJson(res, http.getString());
  if (!ok) Serial.printf("API %s failed: HTTP %d (%s)\n", (const char*)(req["action"] | ""), code, http.errorToString(code).c_str());
  http.end();
  return ok;
}

// Returns matched slot, 0 for an unknown finger, -1 when no finger is on the sensor.
int scanFinger() {
  if (finger.getImage() != FINGERPRINT_OK) return -1;
  if (finger.image2Tz() != FINGERPRINT_OK) return -1;
  if (finger.fingerSearch() != FINGERPRINT_OK) return 0;
  return finger.fingerID;
}

bool waitFinger(uint8_t buf, unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (finger.getImage() == FINGERPRINT_OK) return finger.image2Tz(buf) == FINGERPRINT_OK;
    delay(50);
  }
  return false;
}

bool enroll(int slot) {
  Serial.printf("Enroll slot %d: place finger\n", slot);
  if (!waitFinger(1, 20000)) return false;
  Serial.println("Remove finger");
  unsigned long t0 = millis();
  while (finger.getImage() != FINGERPRINT_NOFINGER && millis() - t0 < 10000) delay(50);
  Serial.println("Place the same finger again");
  if (!waitFinger(2, 20000)) return false;
  return finger.createModel() == FINGERPRINT_OK && finger.storeModel(slot) == FINGERPRINT_OK;
}

// Run one feeder motor until the bin gains `target` grams; returns grams actually given.
// ponytail: on/off control. Add PWM ramp-down near target if auger overshoot can't be tuned out with STOP_EARLY_G.
float dispenseItem(int h, long target) {
  scale.tare(10);
  syncButtons();
  digitalWrite(HOPPER_PINS[h], HIGH);  // motor run
  unsigned long t0 = millis(), lastLcd = 0;
  // STOP button stops the motor early and bills only what was given.
  for (;;) {
    float w = scale.get_units(1);
    if (w >= target - STOP_EARLY_G[h] || millis() - t0 >= ITEM_TIMEOUT_MS || flipped(BTN_STOP, lastStop)) break;
    if (millis() - lastLcd >= 250) {  // live weight on line 2; throttled, I2C writes are slow
      lastLcd = millis();
      char l2[17]; snprintf(l2, sizeof(l2), "%4ld/%ldg      ", lround(w > 0 ? w : 0), target);
      lcd.setCursor(0, 1); lcd.print(l2);
    }
    delay(5);
  }
  digitalWrite(HOPPER_PINS[h], LOW);   // motor stop
  delay(1500);  // let the grain settle before the final reading
  float got = scale.get_units(10);
  Serial.printf("Hopper %d: target %ld g, gave %.0f g\n", h, target, got);
  return got > 0 ? got : 0;
}

// Keep retrying until the server has the result, so a Wi-Fi drop never loses a bill.
void report(JsonDocument& req) {
  JsonDocument res;
  while (!(api(req, res) && res["ok"].is<bool>())) {
    Serial.println("Report failed, retrying...");
    delay(2000);
  }
}

void poll() {
  JsonDocument req, res;
  req["action"] = "poll";
  if (!api(req, res)) return;
  const char* job = res["job"] | "";
  int sid = res["session_id"] | 0;

  if (strcmp(job, "dispense") == 0) {
    digitalWrite(LED, HIGH);
    JsonDocument done;
    done["action"] = "done";
    done["session_id"] = sid;
    JsonArray grams = done["grams"].to<JsonArray>();
    bool ended = false;
    for (int h = 0; h < 3; h++) {
      long target = res["grams"][h] | 0;
      if (target <= 0 || ended) { grams.add(0); continue; }
      char l2[17]; snprintf(l2, sizeof(l2), "%ldg NEXT or STOP", target);
      lcd2(HOPPER_NAME[h], l2);
      if (!waitButton()) { ended = true; grams.add(0); continue; }  // STOP skips the rest
      lcd2(HOPPER_NAME[h], "Dispensing...");
      long got = lround(dispenseItem(h, target));
      grams.add(got);
      snprintf(l2, sizeof(l2), "gave %ld g", got);
      lcd2(HOPPER_NAME[h], l2);
    }
    report(done);
    digitalWrite(LED, LOW);
    Serial.println("Dispense complete");
    blink(2);
    lcd2("Quota fulfilled", "Next customer");  // website shows the bill to distributor + customer
    delay(3000);
    lcd2("E-Ration ready", "Place finger");
  } else if (strcmp(job, "enroll") == 0) {
    int slot = res["slot"] | 0;
    JsonDocument out;
    out["action"] = "enrolled";
    out["session_id"] = sid;
    out["slot"] = slot;
    lcd2("Enroll finger", "Place twice");
    out["ok"] = slot > 0 && enroll(slot);
    Serial.println(out["ok"] ? "Enrolled" : "Enroll failed");
    lcd2(out["ok"] ? "Enrolled OK" : "Enroll failed", "");
    report(out);
    lcd2("E-Ration ready", "Place finger");
  }
}

void identify(int id) {
  JsonDocument req, res;
  req["action"] = "identify";
  req["finger_id"] = id;
  if (api(req, res) && res["ok"]) {
    const char* name = res["name"] | "";
    Serial.printf("Matched: %s. Waiting for distributor...\n", name);
    lcd2(name, "Wait distributor");
    blink(1);
  } else {
    Serial.printf("Rejected: %s\n", (const char*)(res["msg"] | "server unreachable"));
    lcd2("Not recognized", res["msg"] | "try again");
    blink(3);
  }
  delay(2000);  // don't re-send while the finger is still on the sensor
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcd2("E-Ration Node", "Starting...");
  for (int h = 0; h < 3; h++) {
    pinMode(HOPPER_PINS[h], OUTPUT);
    digitalWrite(HOPPER_PINS[h], LOW);  // motor off
  }
  scale.begin(HX_DT, HX_SCK);
  scale.set_scale(CALIBRATE ? 1.0 : SCALE_FACTOR);
  scale.tare(20);

  fpSerial.begin(57600, SERIAL_8N1, FP_RX, FP_TX);
  delay(100);
  if (finger.verifyPassword()) Serial.println("Fingerprint sensor OK");
  else Serial.println("Fingerprint sensor not found! Check power/GND/wiring.");

  WiFi.begin(WIFI_SSID, WIFI_PASS);  // ESP32 reconnects on its own after drops
  Serial.print("Connecting to Wi-Fi");
  lcd2("Connecting", "Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nConnected, IP %s\n", WiFi.localIP().toString().c_str());
  digitalWrite(LED, HIGH);  // ready
  lcd2("E-Ration ready", "Place finger");
}

void loop() {
#if CALIBRATE
  Serial.printf("raw: %.0f\n", scale.get_units(10));
  delay(500);
  return;
#endif
  digitalWrite(LED, WiFi.status() == WL_CONNECTED);  // solid ON = connected & ready; off if Wi-Fi drops
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll > POLL_MS) {
    lastPoll = millis();
    poll();
  }
  int id = scanFinger();
  if (id == 0) {
    Serial.println("Unknown finger");
    blink(3);
    delay(1500);
  } else if (id > 0) {
    identify(id);
  }
}
