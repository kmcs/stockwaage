/*
  Gateway: LILYGO T-SIM7080G S3
  - Empfängt Messdaten von (mehreren) Waagen per ESP-NOW
  - Optionaler Debug-Mode über 3-Positions-Schalter:
      * Position DEBUG: startet WLAN-AP + Webserver (Status/Logs abrufen)
      * Position NORMAL: kein AP, nur ESP-NOW + NB-IoT Upload
  - Sendet Daten (Waagen + Gateway-Status wie LTE Signal) an eine Web-URL (HTTP POST JSON)

  WICHTIG: Du musst unten einmalig anpassen:
    - MODE_PINS (3-Positions-Schalter-Pins)
    - MODEM UART Pins (TX/RX) passend zum T-SIM7080G-S3 (siehe Pinout deines Boards)
    - APN + SIM PIN (falls nötig)
    - BACKEND_HOST/BACKEND_PORT/BACKEND_PATH (dein Web-Endpoint)
    - ESP-NOW Kanal (muss Waage & Gateway gleich sein)

  Getestet ist dies als "robuste Vorlage". Je nach LILYGO-Revision sind UART/Schalter-Pins unterschiedlich.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <HX711.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// ---------- USER CONFIG ----------

// HX711 Scale pins
static const int HX711_DOUT_PIN = 35;  // DT
static const int HX711_CLK_PIN = 36;   // SCK

// I2C pins (AHT20 + BMP280) - Adjust to your board's pinout
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 12;  // Changed from 22 (not available on this board)

// ESP-NOW Funkkanal (muss auf allen Geräten gleich sein)
static const uint8_t ESPNOW_CHANNEL = 6;

// Empfangsfenster (Millisekunden) je Wake-Zyklus, in dem Waagen senden dürfen
static const uint32_t RX_WINDOW_MS = 3000;   // 2-5s sind meist genug
static const uint32_t RX_GRACE_MS  = 500;    // kurzer Nachlauf

// Debug-AP (nur im DEBUG-Modus)
static const char* DEBUG_AP_SSID = "WAAGE-GW-DEBUG";
static const char* DEBUG_AP_PASS = "debug1234"; // mind. 8 Zeichen

// Backend HTTP Endpoint (RAW HTTP POST)
static const char* BACKEND_HOST = "example.com";
static const uint16_t BACKEND_PORT = 80;
static const char* BACKEND_PATH = "/api/waage/ingest";

// Mobilfunk / NB-IoT (TinyGSM)
static const char* APN = "iot.1nce.net";  // Beispiel (1NCE). Für Telekom/Vodafone/… anpassen.
static const char* SIM_PIN = "";          // falls SIM-PIN gesetzt ist, z.B. "1234"

// ---------- BOARD PIN CONFIG (ANPASSEN!) ----------

// 3-Positions-Schalter auswerten:
// Viele 3-Pos-Schalter sind als 2 Signale verdrahtet (A/B) => 3 Zustände.
// Setze die Pins auf die GPIOs, an denen der Switch bei deinem Board hängt.
// Wenn du keine Ahnung hast: erst auf "UNUSED" lassen und später mappen.
static const int MODE_PIN_A = -1;  // z.B. 0/2/3/… (ADC/RTC egal)
static const int MODE_PIN_B = -1;  // z.B. 1/4/5/…

// MODEM UART Pins: Bitte mit deinem T-SIM7080G-S3 Pinout abgleichen!
// Manche LILYGO-Boards nutzen z.B. Serial1/Serial2 mit festen Pins.
// Du kannst auch erst debuggen, indem du AT über USB testest.
static const int MODEM_RX_PIN = -1;  // ESP32 RX (empfängt vom Modem TX)
static const int MODEM_TX_PIN = -1;  // ESP32 TX (sendet zum Modem RX)
static const int MODEM_BAUD   = 115200;

// Optional: Modem Power/Reset Pins (falls herausgeführt). Sonst auf -1 lassen.
static const int MODEM_PWRKEY_PIN = -1;
static const int MODEM_RESET_PIN  = -1;

// ---------- END USER CONFIG ----------


// ---------- TinyGSM (optional, nur wenn du Upload per NB-IoT wirklich nutzen willst) ----------
#define TINY_GSM_MODEM_SIM7080
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

HardwareSerial ModemSerial(1);
TinyGsm modem(ModemSerial);
TinyGsmClient gsmClient(modem);

// ---------- HX711 SCALE ----------
HX711 scale;

// ---------- I2C SENSORS ----------
Adafruit_AHTX0 aht20;
Adafruit_BMP280 bmp280;
bool aht20_ready = false;
bool bmp280_ready = false;

// Sensor data cache
struct SensorData {
  float temp_aht_c = 0;
  float rh_aht = 0;
  float temp_bmp_c = 0;
  float pressure_pa = 0;
  float altitude_m = 0;
} sensor_data;

// ---------- WEB SERVER (Debug AP) ----------
WebServer server(80);

// ---------- DATA STRUCTURES ----------
#pragma pack(push, 1)
struct ScalePacket {
  uint8_t  version;       // packet version
  uint32_t device_id;     // eindeutige ID der Waage
  int32_t  weight_g;      // Gewicht in Gramm
  int16_t  temp_c_x100;   // Temperatur in °C *100 (optional)
  uint16_t rh_x100;       // Luftfeuchte % *100 (optional)
  uint16_t p_hpa_x10;     // Druck hPa *10 (optional)
  uint16_t vbat_mv;       // Batterie der Waage in mV (optional)
  uint32_t seq;           // Sequenznummer
  uint32_t crc32;         // einfache Integritätsprüfung (optional)
};

struct AckPacket {
  uint8_t  version;
  uint32_t gateway_uptime_s;
  uint32_t epoch_s;       // Gateway-Zeit (0 wenn unbekannt)
  uint32_t last_seq_seen; // Bestätigung
};
#pragma pack(pop)

struct LastSeen {
  bool     valid = false;
  uint32_t device_id = 0;
  uint32_t seq = 0;
  int32_t  weight_g = 0;
  int16_t  temp_c_x100 = 0;
  uint16_t rh_x100 = 0;
  uint16_t p_hpa_x10 = 0;
  uint16_t vbat_mv = 0;
  uint32_t received_ms = 0;
  uint8_t  mac[6] = {0};
};

static const int MAX_DEVICES = 10;
LastSeen devices[MAX_DEVICES];

volatile bool got_any_packet = false;
volatile uint32_t packets_received = 0;

static int last_csq = -1;   // LTE signal quality (CSQ)
static int last_rssi = 0;   // derived
static String last_modem_info;

// ---------- UTILS ----------
static String macToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

static int findOrAllocDeviceSlot(uint32_t device_id) {
  int freeSlot = -1;
  for (int i=0;i<MAX_DEVICES;i++) {
    if (devices[i].valid && devices[i].device_id == device_id) return i;
    if (!devices[i].valid && freeSlot < 0) freeSlot = i;
  }
  return freeSlot;
}

// Simple mode decoding for a 3-position switch using two pins.
// Returns: 0=NORMAL, 1=DEBUG, 2=RESERVED
static int readMode() {
  if (MODE_PIN_A < 0 || MODE_PIN_B < 0) {
    return 1; // FORCE DEBUG MODE if pins not configured (temporary for testing)
  }
  // Expecting pullups and switch pulling one/both to GND.
  int a = digitalRead(MODE_PIN_A) == LOW ? 1 : 0;
  int b = digitalRead(MODE_PIN_B) == LOW ? 1 : 0;

  // Map (a,b) -> mode. Adjust if your wiring differs.
  // 00: NORMAL
  // 10: DEBUG
  // 01: RESERVED
  // 11: NORMAL (fallback)
  if (a==1 && b==0) return 1;   // DEBUG
  if (a==0 && b==1) return 2;   // RESERVED
  return 0;                     // NORMAL
}

// ---------- ESP-NOW SEND ACK ----------
static bool ensurePeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

static void sendAck(const uint8_t* mac, uint32_t lastSeq);
static int32_t scaleReadGrams();
static void sensorsRead();

static void sendAck(const uint8_t* mac, uint32_t lastSeq) {
  if (!ensurePeer(mac)) return;

  AckPacket ack{};
  ack.version = 1;
  ack.gateway_uptime_s = (uint32_t)(millis() / 1000);
  ack.epoch_s = 0; // optional: set if you sync time
  ack.last_seq_seen = lastSeq;

  esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
}

// ---------- ESP-NOW RX CALLBACK ----------
void onEspNowRecv(const uint8_t *mac, const uint8_t* data, int len) {
  if (!mac || !data || len < (int)sizeof(ScalePacket)) return;

  ScalePacket pkt;
  memcpy(&pkt, data, sizeof(ScalePacket));

  // Basic sanity checks
  if (pkt.version == 0 || pkt.device_id == 0) return;

  int slot = findOrAllocDeviceSlot(pkt.device_id);
  if (slot < 0) return;

  devices[slot].valid = true;
  devices[slot].device_id = pkt.device_id;
  devices[slot].seq = pkt.seq;
  devices[slot].weight_g = pkt.weight_g;
  devices[slot].temp_c_x100 = pkt.temp_c_x100;
  devices[slot].rh_x100 = pkt.rh_x100;
  devices[slot].p_hpa_x10 = pkt.p_hpa_x10;
  devices[slot].vbat_mv = pkt.vbat_mv;
  devices[slot].received_ms = millis();
  memcpy(devices[slot].mac, mac, 6);

  got_any_packet = true;
  packets_received++;

  // ACK back quickly
  sendAck(mac, pkt.seq);
}

// ---------- DEBUG WEB SERVER ----------
static String buildStatusJson() {
  sensorsRead();  // Read sensors before building JSON
  
  String s = "{";
  s += "\"uptime_s\":" + String((uint32_t)(millis()/1000)) + ",";
  s += "\"packets_received\":" + String((uint32_t)packets_received) + ",";
  s += "\"scale_weight_g\":" + String(scaleReadGrams()) + ",";
  s += "\"sensors\":{";
  s += "\"aht10_temp_c\":" + String(sensor_data.temp_aht_c, 2) + ",";
  s += "\"aht10_rh\":" + String(sensor_data.rh_aht, 1) + ",";
  s += "\"bmp280_temp_c\":" + String(sensor_data.temp_bmp_c, 2) + ",";
  s += "\"bmp280_pressure_pa\":" + String((int)sensor_data.pressure_pa) + ",";
  s += "\"bmp280_altitude_m\":" + String(sensor_data.altitude_m, 1);
  s += "},";
  s += "\"lte_csq\":" + String(last_csq) + ",";
  s += "\"lte_info\":\"" + last_modem_info + "\",";
  s += "\"devices\":[";
  bool first = true;
  for (int i=0;i<MAX_DEVICES;i++) {
    if (!devices[i].valid) continue;
    if (!first) s += ",";
    first = false;
    s += "{";
    s += "\"device_id\":" + String(devices[i].device_id) + ",";
    s += "\"seq\":" + String(devices[i].seq) + ",";
    s += "\"weight_g\":" + String(devices[i].weight_g) + ",";
    s += "\"temp_c\":" + String(devices[i].temp_c_x100 / 100.0f, 2) + ",";
    s += "\"rh\":" + String(devices[i].rh_x100 / 100.0f, 2) + ",";
    s += "\"p_hpa\":" + String(devices[i].p_hpa_x10 / 10.0f, 1) + ",";
    s += "\"vbat_mv\":" + String(devices[i].vbat_mv) + ",";
    s += "\"last_ms_ago\":" + String((uint32_t)(millis() - devices[i].received_ms)) + ",";
    s += "\"mac\":\"" + macToString(devices[i].mac) + "\"";
    s += "}";
  }
  s += "]}";
  return s;
}

static void startDebugApAndServer() {
  WiFi.mode(WIFI_AP_STA); // AP for debug + STA side for ESP-NOW
  WiFi.softAP(DEBUG_AP_SSID, DEBUG_AP_PASS, ESPNOW_CHANNEL, 0, 4);

  server.on("/", [](){
    server.send(200, "application/json", buildStatusJson());
  });
  server.on("/reboot", [](){
    server.send(200, "text/plain", "rebooting");
    delay(200);
    ESP.restart();
  });
  server.begin();
}

// ---------- ESP-NOW INIT ----------
static bool initEspNow() {
  // ESP-NOW needs WiFi initialized
  WiFi.mode(WIFI_STA); // minimal, works for ESP-NOW
  WiFi.disconnect(true, true);

  // Force channel (important when also using AP or when peer fixed channel)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onEspNowRecv);
  return true;
}

// ---------- SCALE HELPERS ----------
static void scaleInit() {
  Serial.println("Initializing HX711 scale...");
  scale.begin(HX711_DOUT_PIN, HX711_CLK_PIN);
  
  // Optional: calibrate or set a known scale factor
  // You need to calibrate this for your setup:
  // 1. Measure the raw value with 0g (tare)
  // 2. Measure the raw value with known weight (e.g., 1kg)
  // 3. Calculate: scale_factor = (raw_1kg - raw_0g) / 1000
  
  scale.set_scale(420.0);  // Adjust this value based on your calibration
  scale.tare();            // Reset to 0
  Serial.println("Scale ready!");
}

static int32_t scaleReadGrams() {
  if (scale.is_ready()) {
    long rawValue = scale.get_units(10);  // Average of 10 readings
    return (int32_t)rawValue;
  }
  return 0;
}

// ---------- SENSOR HELPERS (AHT20 + BMP280) ----------
static void sensorsInit() {
  Serial.println("Initializing I2C sensors...");
  
  // Initialize I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(100);
  Serial.printf("I2C initialized on SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.flush();
  
  // Initialize AHT20
  Serial.print("Scanning for AHT20 at 0x38...");
  if (aht20.begin()) {  // Adafruit AHTX0 uses default I2C address 0x38
    aht20_ready = true;
    Serial.println(" Found!");
  } else {
    Serial.println(" Not found!");
  }
  Serial.flush();
  
  // Initialize BMP280 - try both addresses
  Serial.print("Scanning for BMP280 at 0x76...");
  if (bmp280.begin(0x76)) {  // 0x76 is default I2C address for BMP280
    bmp280_ready = true;
    Serial.println(" Found!");
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                       Adafruit_BMP280::SAMPLING_X2,   // temperature
                       Adafruit_BMP280::SAMPLING_X16,  // pressure
                       Adafruit_BMP280::FILTER_X16,
                       Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 configured!");
  } else {
    Serial.println(" Not found, trying 0x77...");
    if (bmp280.begin(0x77)) {
      bmp280_ready = true;
      Serial.println("BMP280 found at 0x77!");
      bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                         Adafruit_BMP280::SAMPLING_X2,
                         Adafruit_BMP280::SAMPLING_X16,
                         Adafruit_BMP280::FILTER_X16,
                         Adafruit_BMP280::STANDBY_MS_500);
      Serial.println("BMP280 configured!");
    } else {
      Serial.println("BMP280 not found at either address!");
    }
  }
  Serial.flush();
}

static void sensorsRead() {
  // Read AHT20
  if (aht20_ready) {
    sensors_event_t humidity, temp;
    aht20.getEvent(&humidity, &temp);
    sensor_data.temp_aht_c = temp.temperature;
    sensor_data.rh_aht = humidity.relative_humidity;
  }
  
  // Read BMP280
  if (bmp280_ready) {
    sensor_data.temp_bmp_c = bmp280.readTemperature();
    sensor_data.pressure_pa = bmp280.readPressure();
    sensor_data.altitude_m = bmp280.readAltitude(1013.25);  // Sea level pressure
  }
}

// ---------- MODEM HELPERS ----------
static void modemPinsInit() {
  if (MODEM_PWRKEY_PIN >= 0) {
    pinMode(MODEM_PWRKEY_PIN, OUTPUT);
    digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  }
  if (MODEM_RESET_PIN >= 0) {
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, HIGH);
  }
}

static bool modemBegin() {
  if (MODEM_RX_PIN < 0 || MODEM_TX_PIN < 0) {
    // Not configured, skip modem use
    last_modem_info = "MODEM UART pins not set";
    return false;
  }

  ModemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  delay(200);

  modemPinsInit();

  // Basic init
  if (!modem.init()) {
    last_modem_info = "modem.init failed";
    return false;
  }

  // SIM unlock if needed
  if (strlen(SIM_PIN) > 0) {
    if (modem.getSimStatus() != 3) { // 3 = READY in TinyGSM
      modem.simUnlock(SIM_PIN);
    }
  }

  // Network mode: let modem decide; you can force NB-IoT/LTE-M via AT if needed.

  // Attach
  modem.gprsConnect(APN, "", "");
  // Note: name "gprsConnect" in TinyGSM also used for LTE-M/NB-IoT profiles.

  // Read modem info + signal
  last_modem_info = modem.getModemInfo();
  last_csq = modem.getSignalQuality(); // 0..31, 99 unknown

  return true;
}

static void modemEnd() {
  // Best-effort shutdown: disconnect PDP and sleep.
  // Some setups prefer AT+CFUN=0 or PSM, but depends on your SIM/provider.
  modem.gprsDisconnect();
  ModemSerial.end();
}

// ---------- BACKEND UPLOAD (RAW HTTP POST) ----------
static String buildUploadJson() {
  // Minimal JSON: gateway stats + array of device readings
  String s = "{";
  s += "\"gateway\":{";
  s += "\"uptime_s\":" + String((uint32_t)(millis()/1000)) + ",";
  s += "\"lte_csq\":" + String(last_csq) + ",";
  s += "\"modem\":\"" + last_modem_info + "\"";
  s += "},";
  s += "\"scales\":[";
  bool first = true;
  for (int i=0;i<MAX_DEVICES;i++) {
    if (!devices[i].valid) continue;
    if (!first) s += ",";
    first = false;
    s += "{";
    s += "\"device_id\":" + String(devices[i].device_id) + ",";
    s += "\"seq\":" + String(devices[i].seq) + ",";
    s += "\"weight_g\":" + String(devices[i].weight_g) + ",";
    s += "\"temp_c\":" + String(devices[i].temp_c_x100 / 100.0f, 2) + ",";
    s += "\"rh\":" + String(devices[i].rh_x100 / 100.0f, 2) + ",";
    s += "\"p_hpa\":" + String(devices[i].p_hpa_x10 / 10.0f, 1) + ",";
    s += "\"vbat_mv\":" + String(devices[i].vbat_mv) + ",";
    s += "\"mac\":\"" + macToString(devices[i].mac) + "\"";
    s += "}";
  }
  s += "]}";
  return s;
}

static bool uploadToBackendOverCellular() {
  String body = buildUploadJson();

  if (!gsmClient.connect(BACKEND_HOST, BACKEND_PORT)) {
    return false;
  }

  String req;
  req += "POST " + String(BACKEND_PATH) + " HTTP/1.1\r\n";
  req += "Host: " + String(BACKEND_HOST) + "\r\n";
  req += "User-Agent: T-SIM7080G-S3-Gateway\r\n";
  req += "Connection: close\r\n";
  req += "Content-Type: application/json\r\n";
  req += "Content-Length: " + String(body.length()) + "\r\n\r\n";
  req += body;

  gsmClient.print(req);

  // Simple response drain (optional)
  uint32_t start = millis();
  while (gsmClient.connected() && (millis() - start) < 5000) {
    while (gsmClient.available()) {
      gsmClient.read();
    }
    delay(10);
  }
  gsmClient.stop();
  return true;
}

// ---------- SLEEP SCHEDULING ----------
static uint64_t secondsToNextWake_30min_dayOnly() {
  // Keine RTC: Wir wachen einfach alle 30 Minuten (tagsüber) und nachts alle X Stunden.
  // Da du nachts keine Updates brauchst: nachts längerer Sleep.
  // Ohne echte Uhr kann man "06-20" nicht exakt. Zwei Wege:
  //   A) Simple: IMMER alle 30 min (stabil, aber mehr Verbrauch)
  //   B) "Pseudo-night": wenn länger keine Pakete kamen, länger schlafen (nicht perfekt)
  //
  // Hier implementiere ich A als sicherer Default:
  // -> alle 30 Minuten
  return 30ULL * 60ULL;
}

static void goDeepSleepSeconds(uint64_t seconds) {
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
  esp_deep_sleep_start();
}

// ---------- SETUP / LOOP ----------
void setup() {
  // Extra delay for USB to stabilize
  for(int i=0; i<5; i++) {
    delay(500);
  }
  
  Serial.begin(115200);
  delay(1000);
  
  // Force early output to be visible
  Serial.write('\n');
  Serial.write('\n');
  Serial.write('\n');
  for (int i = 0; i < 3; i++) {
    Serial.println("\n");
    delay(100);
  }
  Serial.println("========== STARTUP ==========");
  Serial.println("Hello! Device is booting...");
  Serial.flush();

  // Mode pins
  if (MODE_PIN_A >= 0) pinMode(MODE_PIN_A, INPUT_PULLUP);
  if (MODE_PIN_B >= 0) pinMode(MODE_PIN_B, INPUT_PULLUP);

  int mode = readMode();
  Serial.printf("Mode: %d\n", mode);
  Serial.flush();

  // Init ESP-NOW always (both modes)
  Serial.println("Initializing ESP-NOW...");
  Serial.flush();
  if (!initEspNow()) {
    Serial.println("ESP-NOW init failed");
  } else {
    Serial.println("ESP-NOW ready");
  }
  Serial.flush();

  // Initialize scale
  Serial.println("Initializing scale...");
  Serial.flush();
  scaleInit();
  Serial.flush();

  // Initialize sensors
  Serial.println("Initializing sensors...");
  Serial.flush();
  sensorsInit();
  Serial.flush();

  if (mode == 1) {
    // DEBUG: AP + Webserver, stay awake
    Serial.println("Starting DEBUG mode...");
    Serial.flush();
    startDebugApAndServer();
    Serial.println("DEBUG mode: AP + web server started");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    return;
  }

  // NORMAL mode: short RX window, then cellular upload, then sleep

  // Listen for ESP-NOW packets for RX_WINDOW_MS
  uint32_t t0 = millis();
  while (millis() - t0 < RX_WINDOW_MS) {
    delay(10);
  }
  // small grace
  delay(RX_GRACE_MS);

  // If we received anything, forward it. If not, you can still report gateway health.
  bool modemOk = false;
  if (MODEM_RX_PIN >= 0 && MODEM_TX_PIN >= 0) {
    Serial.println("Modem pins configured, initializing...");
    modemOk = modemBegin();
    if (modemOk) {
      bool ok = uploadToBackendOverCellular();
      Serial.println(ok ? "Upload OK" : "Upload FAILED");
      modemEnd();
    } else {
      Serial.println("Modem failed to initialize");
    }
  } else {
    Serial.println("Modem pins NOT configured (-1), skipping modem");
  }

  // Sleep until next slot
  uint64_t sleep_s = secondsToNextWake_30min_dayOnly();
  Serial.printf("Sleeping for %llu seconds\n", sleep_s);
  delay(50);
  goDeepSleepSeconds(sleep_s);
}

void loop() {
  // DEBUG mode only: serve web
  server.handleClient();
  delay(10);
}
