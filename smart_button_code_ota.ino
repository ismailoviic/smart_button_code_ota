#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_arduino_version.h>

#include <SPI.h>
#include <MFRC522.h>

// =====================================================
// FIRMWARE VERSION
// =====================================================

#define FIRMWARE_VERSION "button_v0.2.0"

// =====================================================
// BOOTSTRAP CONFIG
// CHANGE THIS BLOCK ONLY FOR EACH FINAL MANUAL UPLOAD
// =====================================================

#define BOOTSTRAP_DEVICE_ID       "BTN-L1N2"
#define BOOTSTRAP_TARGET_HUB_ID   "HUB-L1"
#define BOOTSTRAP_LINE_ID         "LINE-001"
#define BOOTSTRAP_FACTORY_ID      "FACTORY-001"
#define BOOTSTRAP_HUB_MAC         "F4:65:0B:49:AA:64"

// Set this true for the final manual upload if the device may contain old test config.
// For future universal OTA firmware, set this false.
#define BOOTSTRAP_FORCE_OVERWRITE true

// Direct .bin URL for first OTA tests.
// Later we can make this a manifest.json URL.
#define BOOTSTRAP_FIRMWARE_URL    ""

// =====================================================
// HARDWARE PINS
// =====================================================

#define BUTTON_PIN      2

#define LED_RED_PIN     14
#define LED_GREEN_PIN   27
#define LED_BLUE_PIN    26

#define BATT_PIN        34
#define USB_PIN         35

#define SS_PIN          5
#define RST_PIN         22

// =====================================================
// PILOT NETWORK CONFIG
// =====================================================

#define ESPNOW_CHANNEL 1

#define SETUP_AP_PASSWORD "12345678"

IPAddress setupIP(192, 168, 4, 1);
IPAddress setupGateway(192, 168, 4, 1);
IPAddress setupSubnet(255, 255, 255, 0);

// =====================================================
// BATTERY CALIBRATION
// 2 x 10k voltage divider => multiplier = 2.0
// =====================================================

const float VOLTAGE_MULTIPLIER = 2.0;
const float ADC_REFERENCE_VOLTAGE = 3.3;

const float MAX_BATT_VOLTAGE = 4.2;
const float MIN_BATT_VOLTAGE = 3.2;

#define OTA_MIN_BATTERY_PERCENT 40

// =====================================================
// PROTOCOL
// =====================================================

#define PROTOCOL_VERSION 1

enum MessageType {
  MSG_HELLO = 1,
  MSG_ACK = 2,
  MSG_BADGE_SCAN = 3,
  MSG_PIECE_DONE = 4,
  MSG_BATTERY_STATUS = 5,
  MSG_HEARTBEAT = 6,
  MSG_ERROR = 7,
  MSG_CONFIG_UPDATE = 8,
  MSG_OTA_AVAILABLE = 9,
  MSG_OTA_START = 10,
  MSG_OTA_STATUS = 11
};

typedef struct __attribute__((packed)) {
  uint8_t protocolVersion;
  uint8_t messageType;
  uint8_t batteryPercent;
  uint8_t reserved1;

  uint32_t bootId;
  uint32_t sequence;
  uint32_t uptimeSeconds;

  uint16_t batteryMv;
  uint16_t eventValue;

  char deviceId[16];
  char targetHubId[16];
  char badgeUid[24];
  char firmwareVersion[16];
} ButtonToHubPacket;

typedef struct __attribute__((packed)) {
  uint8_t protocolVersion;
  uint8_t messageType;
  uint8_t ackForMessageType;
  uint8_t statusCode;

  uint32_t ackSequence;
  uint32_t hubUptimeSeconds;

  char hubId[16];
  char message[32];
  char targetVersion[16];
} HubToButtonAck;

// =====================================================
// OBJECTS
// =====================================================

Preferences prefs;
WebServer server(80);
DNSServer dnsServer;

MFRC522 mfrc522(SS_PIN, RST_PIN);

// =====================================================
// CONFIG STRUCT
// =====================================================

struct ButtonConfig {
  String deviceId;
  String factoryId;
  String lineId;
  String targetHubId;
  String targetHubMac;
  String wifiSsid;
  String wifiPassword;
  String firmwareUrl;
  uint32_t configVersion;
};

ButtonConfig config;

uint8_t targetHubMacBytes[6];

// =====================================================
// STATE
// =====================================================

uint32_t bootId = 0;
uint32_t sequenceCounter = 0;
uint16_t pieceCounter = 0;

bool hubConnected = false;
bool waitingForAck = false;

uint32_t pendingAckSequence = 0;
uint8_t pendingAckMessageType = 0;
unsigned long lastAckWaitStartMs = 0;

bool otaRequested = false;
char requestedOtaVersion[16] = "";

bool setupPortalActive = false;

// =====================================================
// TIMERS
// =====================================================

const unsigned long SETUP_HOLD_TIME_MS = 5000;

const unsigned long HELLO_RETRY_INTERVAL_MS = 1000;
const unsigned long ACK_TIMEOUT_MS = 1200;

const unsigned long HEARTBEAT_INTERVAL_MS = 60000;
const unsigned long BATTERY_STATUS_INTERVAL_MS = 600000;

const unsigned long BUTTON_DEBOUNCE_MS = 60;

unsigned long lastHelloAttemptMs = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastBatteryStatusMs = 0;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastButtonChangeMs = 0;

// =====================================================
// LED MANAGER
// =====================================================

enum LedMode {
  LED_BOOT_BLUE_BLINK,
  LED_GREEN_SOLID,
  LED_GREEN_FLASH,
  LED_YELLOW_SOLID,
  LED_YELLOW_BLINK,
  LED_RED_SOLID,
  LED_RED_BLINK,
  LED_PURPLE_SOLID,
  LED_PURPLE_BLINK,
  LED_PINK_SOLID,
  LED_REJECT_RED_BLUE,
  LED_OTA_FAILED
};

LedMode currentLedMode = LED_BOOT_BLUE_BLINK;

unsigned long ledLastChangeMs = 0;
unsigned long ledModeUntilMs = 0;
bool ledOn = false;
bool ledAlt = false;

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_RED_PIN, r ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, g ? HIGH : LOW);
  digitalWrite(LED_BLUE_PIN, b ? HIGH : LOW);
}

void applyColor(bool on, bool r, bool g, bool b) {
  if (!on) {
    setLED(false, false, false);
  } else {
    setLED(r, g, b);
  }
}

void setLedMode(LedMode mode, unsigned long durationMs = 0) {
  currentLedMode = mode;
  ledLastChangeMs = 0;
  ledOn = true;
  ledAlt = false;

  if (durationMs > 0) {
    ledModeUntilMs = millis() + durationMs;
  } else {
    ledModeUntilMs = 0;
  }
}

void updateLed() {
  unsigned long now = millis();

  if (ledModeUntilMs > 0 && now > ledModeUntilMs) {
    if (hubConnected) {
      setLedMode(LED_GREEN_SOLID);
    } else {
      setLedMode(LED_YELLOW_BLINK);
    }
    return;
  }

  switch (currentLedMode) {
    case LED_BOOT_BLUE_BLINK:
      if (now - ledLastChangeMs >= (ledOn ? 300 : 200)) {
        ledLastChangeMs = now;
        ledOn = !ledOn;
      }
      applyColor(ledOn, false, false, true);
      break;

    case LED_GREEN_SOLID:
      setLED(false, true, false);
      break;

    case LED_GREEN_FLASH:
      if (now - ledLastChangeMs >= (ledOn ? 300 : 200)) {
        ledLastChangeMs = now;
        ledOn = !ledOn;
      }
      applyColor(ledOn, false, true, false);
      break;

    case LED_YELLOW_SOLID:
      setLED(true, true, false);
      break;

    case LED_YELLOW_BLINK:
      if (now - ledLastChangeMs >= (ledOn ? 300 : 200)) {
        ledLastChangeMs = now;
        ledOn = !ledOn;
      }
      applyColor(ledOn, true, true, false);
      break;

    case LED_RED_SOLID:
      setLED(true, false, false);
      break;

    case LED_RED_BLINK:
    case LED_OTA_FAILED:
      if (now - ledLastChangeMs >= 250) {
        ledLastChangeMs = now;
        ledOn = !ledOn;
      }
      applyColor(ledOn, true, false, false);
      break;

    case LED_PURPLE_SOLID:
      setLED(true, false, true);
      break;

    case LED_PURPLE_BLINK:
      if (now - ledLastChangeMs >= 250) {
        ledLastChangeMs = now;
        ledOn = !ledOn;
      }
      applyColor(ledOn, true, false, true);
      break;

    case LED_PINK_SOLID:
      setLED(true, false, true);
      break;

    case LED_REJECT_RED_BLUE:
      if (now - ledLastChangeMs >= 200) {
        ledLastChangeMs = now;
        ledAlt = !ledAlt;
      }
      if (ledAlt) {
        setLED(true, false, false);
      } else {
        setLED(false, false, true);
      }
      break;
  }
}

// =====================================================
// HELPERS
// =====================================================

void safeCopy(char *dest, const char *src, size_t maxLen) {
  strncpy(dest, src, maxLen);
  dest[maxLen - 1] = '\0';
}

String macToString(const uint8_t *mac) {
  char buffer[18];
  snprintf(
    buffer,
    sizeof(buffer),
    "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
  return String(buffer);
}

bool parseMacAddress(const String &macStr, uint8_t *mac) {
  int values[6];

  if (sscanf(
        macStr.c_str(),
        "%x:%x:%x:%x:%x:%x",
        &values[0], &values[1], &values[2],
        &values[3], &values[4], &values[5]
      ) != 6) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    if (values[i] < 0 || values[i] > 255) return false;
    mac[i] = (uint8_t)values[i];
  }

  return true;
}

String htmlEscape(String value) {
  value.replace("&", "&amp;");
  value.replace("<", "&lt;");
  value.replace(">", "&gt;");
  value.replace("\"", "&quot;");
  value.replace("'", "&#39;");
  return value;
}

const char* messageTypeToText(uint8_t type) {
  switch (type) {
    case MSG_HELLO: return "HELLO";
    case MSG_ACK: return "ACK";
    case MSG_BADGE_SCAN: return "BADGE_SCAN";
    case MSG_PIECE_DONE: return "PIECE_DONE";
    case MSG_BATTERY_STATUS: return "BATTERY_STATUS";
    case MSG_HEARTBEAT: return "HEARTBEAT";
    case MSG_ERROR: return "ERROR";
    case MSG_CONFIG_UPDATE: return "CONFIG_UPDATE";
    case MSG_OTA_AVAILABLE: return "OTA_AVAILABLE";
    case MSG_OTA_START: return "OTA_START";
    case MSG_OTA_STATUS: return "OTA_STATUS";
    default: return "UNKNOWN";
  }
}

// =====================================================
// BATTERY
// =====================================================

uint16_t readBatteryMv() {
  long total = 0;
  const int samples = 20;

  for (int i = 0; i < samples; i++) {
    total += analogRead(BATT_PIN);
    delayMicroseconds(500);
  }

  float avgAdc = total / (float)samples;
  float pinVoltage = (avgAdc / 4095.0) * ADC_REFERENCE_VOLTAGE;
  float batteryVoltage = pinVoltage * VOLTAGE_MULTIPLIER;

  return (uint16_t)(batteryVoltage * 1000.0);
}

uint8_t batteryMvToPercent(uint16_t mv) {
  float voltage = mv / 1000.0;

  if (voltage >= MAX_BATT_VOLTAGE) return 100;
  if (voltage <= MIN_BATT_VOLTAGE) return 0;

  float percent = ((voltage - MIN_BATT_VOLTAGE) / (MAX_BATT_VOLTAGE - MIN_BATT_VOLTAGE)) * 100.0;

  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  return (uint8_t)percent;
}

bool isUsbPlugged() {
  return digitalRead(USB_PIN) == HIGH;
}

// =====================================================
// CONFIG MANAGER
// =====================================================

void saveConfigToNVS() {
  prefs.begin("btn_config", false);

  prefs.putString("device_id", config.deviceId);
  prefs.putString("factory_id", config.factoryId);
  prefs.putString("line_id", config.lineId);
  prefs.putString("hub_id", config.targetHubId);
  prefs.putString("hub_mac", config.targetHubMac);
  prefs.putString("wifi_ssid", config.wifiSsid);
  prefs.putString("wifi_pass", config.wifiPassword);
  prefs.putString("fw_url", config.firmwareUrl);
  prefs.putUInt("cfg_ver", config.configVersion);
  prefs.putBool("initialized", true);

  prefs.end();

  Serial.println("[CONFIG] Saved to NVS.");
}

void saveBootstrapConfigToNVS() {
  config.deviceId = BOOTSTRAP_DEVICE_ID;
  config.factoryId = BOOTSTRAP_FACTORY_ID;
  config.lineId = BOOTSTRAP_LINE_ID;
  config.targetHubId = BOOTSTRAP_TARGET_HUB_ID;
  config.targetHubMac = BOOTSTRAP_HUB_MAC;
  config.wifiSsid = "";
  config.wifiPassword = "";
  config.firmwareUrl = BOOTSTRAP_FIRMWARE_URL;
  config.configVersion = 1;

  saveConfigToNVS();
}

void loadConfigFromNVS() {
  prefs.begin("btn_config", false);

  bool initialized = prefs.getBool("initialized", false);

  prefs.end();

  if (!initialized || BOOTSTRAP_FORCE_OVERWRITE) {
    Serial.println("[CONFIG] No config found or bootstrap overwrite enabled.");
    Serial.println("[CONFIG] Saving bootstrap config.");
    saveBootstrapConfigToNVS();
  }

  prefs.begin("btn_config", true);

  config.deviceId = prefs.getString("device_id", BOOTSTRAP_DEVICE_ID);
  config.factoryId = prefs.getString("factory_id", BOOTSTRAP_FACTORY_ID);
  config.lineId = prefs.getString("line_id", BOOTSTRAP_LINE_ID);
  config.targetHubId = prefs.getString("hub_id", BOOTSTRAP_TARGET_HUB_ID);
  config.targetHubMac = prefs.getString("hub_mac", BOOTSTRAP_HUB_MAC);
  config.wifiSsid = prefs.getString("wifi_ssid", "");
  config.wifiPassword = prefs.getString("wifi_pass", "");
  config.firmwareUrl = prefs.getString("fw_url", BOOTSTRAP_FIRMWARE_URL);
  config.configVersion = prefs.getUInt("cfg_ver", 1);

  prefs.end();

  Serial.println();
  Serial.println("========== LOADED CONFIG ==========");
  Serial.print("Device ID: "); Serial.println(config.deviceId);
  Serial.print("Factory ID: "); Serial.println(config.factoryId);
  Serial.print("Line ID: "); Serial.println(config.lineId);
  Serial.print("Target Hub ID: "); Serial.println(config.targetHubId);
  Serial.print("Target Hub MAC: "); Serial.println(config.targetHubMac);
  Serial.print("Wi-Fi SSID: "); Serial.println(config.wifiSsid);
  Serial.print("Firmware URL: "); Serial.println(config.firmwareUrl);
  Serial.print("Config Version: "); Serial.println(config.configVersion);
  Serial.println("===================================");
}

// =====================================================
// CONFIG PORTAL
// =====================================================

String buildConfigPage() {
  String html = "";

  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESS Smart Button Setup</title>";

  html += "<style>";
  html += "body{font-family:Arial;background:#f4f4f4;margin:0;padding:20px;}";
  html += ".card{background:white;max-width:520px;margin:auto;padding:22px;border-radius:14px;box-shadow:0 2px 10px rgba(0,0,0,.16);}";
  html += "h2{text-align:center;margin-top:0;}";
  html += "label{font-weight:bold;margin-top:12px;display:block;}";
  html += "input{width:100%;padding:12px;margin-top:6px;box-sizing:border-box;border:1px solid #ccc;border-radius:7px;}";
  html += "button{width:100%;padding:14px;margin-top:20px;background:#111;color:white;border:none;border-radius:7px;font-size:16px;}";
  html += ".info{background:#eee;padding:12px;border-radius:8px;margin-bottom:16px;}";
  html += ".small{font-size:13px;color:#666;}";
  html += "</style>";

  html += "</head><body><div class='card'>";

  html += "<h2>ESS Smart Button Setup</h2>";

  html += "<div class='info'>";
  html += "<b>Firmware:</b> ";
  html += FIRMWARE_VERSION;
  html += "<br><b>Setup IP:</b> 192.168.4.1";
  html += "<br><b>Current MAC:</b> ";
  html += WiFi.macAddress();
  html += "</div>";

  html += "<form action='/save' method='POST'>";

  html += "<label>Device ID</label>";
  html += "<input name='device_id' value='" + htmlEscape(config.deviceId) + "'>";

  html += "<label>Factory ID</label>";
  html += "<input name='factory_id' value='" + htmlEscape(config.factoryId) + "'>";

  html += "<label>Line ID</label>";
  html += "<input name='line_id' value='" + htmlEscape(config.lineId) + "'>";

  html += "<label>Target Hub ID</label>";
  html += "<input name='target_hub_id' value='" + htmlEscape(config.targetHubId) + "'>";

  html += "<label>Target Hub MAC</label>";
  html += "<input name='target_hub_mac' placeholder='F4:65:0B:49:AA:64' value='" + htmlEscape(config.targetHubMac) + "'>";

  html += "<label>Factory Wi-Fi SSID</label>";
  html += "<input name='wifi_ssid' value='" + htmlEscape(config.wifiSsid) + "'>";

  html += "<label>Factory Wi-Fi Password</label>";
  html += "<input type='password' name='wifi_password' value='" + htmlEscape(config.wifiPassword) + "'>";

  html += "<label>Firmware Manifest / Direct .bin URL</label>";
  html += "<input name='firmware_url' placeholder='https://.../firmware.bin' value='" + htmlEscape(config.firmwareUrl) + "'>";
  html += "<p class='small'>For v0.2.0 this field is treated as a direct firmware .bin URL.</p>";

  html += "<button type='submit'>Save & Restart</button>";
  html += "</form>";

  html += "</div></body></html>";

  return html;
}

void handleConfigRoot() {
  server.send(200, "text/html", buildConfigPage());
}

void handleConfigSave() {
  config.deviceId = server.arg("device_id");
  config.factoryId = server.arg("factory_id");
  config.lineId = server.arg("line_id");
  config.targetHubId = server.arg("target_hub_id");
  config.targetHubMac = server.arg("target_hub_mac");
  config.targetHubMac.toUpperCase();

  config.wifiSsid = server.arg("wifi_ssid");
  config.wifiPassword = server.arg("wifi_password");
  config.firmwareUrl = server.arg("firmware_url");
  config.configVersion++;

  saveConfigToNVS();

  String html = "";
  html += "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'></head><body>";
  html += "<h2>Configuration Saved</h2>";
  html += "<p>Device will restart now.</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);

  delay(800);
  ESP.restart();
}

void handleCaptivePortal() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

bool shouldEnterSetupPortal() {
  if (digitalRead(BUTTON_PIN) == HIGH) {
    return false;
  }

  Serial.println("[SETUP] Button held during boot. Hold for 5 seconds to enter setup.");

  unsigned long start = millis();

  while (digitalRead(BUTTON_PIN) == LOW) {
    updateLed();

    if (millis() - start >= SETUP_HOLD_TIME_MS) {
      Serial.println("[SETUP] Entering config portal.");
      return true;
    }

    delay(20);
  }

  Serial.println("[SETUP] Button released before 5 seconds. Continue normal boot.");
  return false;
}

void startConfigPortal() {
  setupPortalActive = true;

  setLedMode(LED_PURPLE_SOLID);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(setupIP, setupGateway, setupSubnet);

  // String apSsid = "ESS-" + config.deviceId + "-SETUP";
  String apSsid = config.deviceId;

  WiFi.softAP(apSsid.c_str(), SETUP_AP_PASSWORD);

  dnsServer.start(53, "*", setupIP);

  server.on("/", HTTP_GET, handleConfigRoot);
  server.on("/save", HTTP_POST, handleConfigSave);

  server.on("/generate_204", HTTP_GET, handleConfigRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleConfigRoot);
  server.on("/fwlink", HTTP_GET, handleConfigRoot);

  server.onNotFound(handleCaptivePortal);

  server.begin();

  Serial.println();
  Serial.println("========== CONFIG PORTAL ACTIVE ==========");
  Serial.print("SSID: "); Serial.println(apSsid);
  Serial.print("Password: "); Serial.println(SETUP_AP_PASSWORD);
  Serial.println("URL: http://192.168.4.1");
  Serial.println("==========================================");

  while (true) {
    dnsServer.processNextRequest();
    server.handleClient();
    updateLed();
    delay(2);
  }
}

// =====================================================
// ESP-NOW SEND
// =====================================================

bool sendButtonPacket(uint8_t messageType, const char *badgeUid, uint16_t eventValue) {
  if (waitingForAck && messageType != MSG_HELLO) {
    Serial.println("[BTN] Busy waiting for ACK. Packet skipped.");
    return false;
  }

  ButtonToHubPacket packet;

  uint16_t batteryMv = readBatteryMv();
  uint8_t batteryPercent = batteryMvToPercent(batteryMv);

  packet.protocolVersion = PROTOCOL_VERSION;
  packet.messageType = messageType;
  packet.batteryPercent = batteryPercent;
  packet.reserved1 = 0;

  packet.bootId = bootId;
  packet.sequence = ++sequenceCounter;
  packet.uptimeSeconds = millis() / 1000;

  packet.batteryMv = batteryMv;
  packet.eventValue = eventValue;

  safeCopy(packet.deviceId, config.deviceId.c_str(), sizeof(packet.deviceId));
  safeCopy(packet.targetHubId, config.targetHubId.c_str(), sizeof(packet.targetHubId));

  if (badgeUid != nullptr) {
    safeCopy(packet.badgeUid, badgeUid, sizeof(packet.badgeUid));
  } else {
    safeCopy(packet.badgeUid, "", sizeof(packet.badgeUid));
  }

  safeCopy(packet.firmwareVersion, FIRMWARE_VERSION, sizeof(packet.firmwareVersion));

  Serial.println();
  Serial.println("========== SEND PACKET ==========");
  Serial.print("Type: "); Serial.println(messageTypeToText(messageType));
  Serial.print("Device: "); Serial.println(packet.deviceId);
  Serial.print("Hub: "); Serial.println(packet.targetHubId);
  Serial.print("Hub MAC: "); Serial.println(macToString(targetHubMacBytes));
  Serial.print("Sequence: "); Serial.println(packet.sequence);
  Serial.print("Battery: "); Serial.print(packet.batteryMv); Serial.print("mV / ");
  Serial.print(packet.batteryPercent); Serial.println("%");
  Serial.print("Firmware: "); Serial.println(packet.firmwareVersion);
  if (messageType == MSG_BADGE_SCAN) {
    Serial.print("Badge UID: "); Serial.println(packet.badgeUid);
  }
  Serial.println("===============================");

  waitingForAck = true;
  pendingAckSequence = packet.sequence;
  pendingAckMessageType = messageType;
  lastAckWaitStartMs = millis();

  esp_err_t result = esp_now_send(targetHubMacBytes, (uint8_t *)&packet, sizeof(packet));

  if (result == ESP_OK) {
    Serial.println("[BTN] ESP-NOW send request OK.");
    return true;
  }

  Serial.print("[BTN] ESP-NOW send request failed. Error: ");
  Serial.println(result);

  waitingForAck = false;
  setLedMode(LED_YELLOW_BLINK, 2000);
  return false;
}

// =====================================================
// ESP-NOW CALLBACKS
// =====================================================

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("[BTN] Send callback: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "LOW-LEVEL SUCCESS" : "LOW-LEVEL FAILED");

  if (status != ESP_NOW_SEND_SUCCESS) {
    setLedMode(LED_YELLOW_BLINK, 2000);
  }
}
#else
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  Serial.print("[BTN] Send callback to ");
  Serial.print(macToString(macAddr));
  Serial.print(": ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "LOW-LEVEL SUCCESS" : "LOW-LEVEL FAILED");

  if (status != ESP_NOW_SEND_SUCCESS) {
    setLedMode(LED_YELLOW_BLINK, 2000);
  }
}
#endif

void handleHubPacket(const uint8_t *senderMac, const uint8_t *incomingData, int len) {
  if (len != sizeof(HubToButtonAck)) {
    Serial.print("[BTN] Invalid incoming packet size: ");
    Serial.println(len);
    return;
  }

  HubToButtonAck packet;
  memcpy(&packet, incomingData, sizeof(packet));

  Serial.println();
  Serial.println("========== HUB PACKET ==========");
  Serial.print("From MAC: "); Serial.println(macToString(senderMac));
  Serial.print("Message Type: "); Serial.println(messageTypeToText(packet.messageType));
  Serial.print("ACK For: "); Serial.println(messageTypeToText(packet.ackForMessageType));
  Serial.print("ACK Sequence: "); Serial.println(packet.ackSequence);
  Serial.print("Hub ID: "); Serial.println(packet.hubId);
  Serial.print("Status: "); Serial.println(packet.statusCode);
  Serial.print("Message: "); Serial.println(packet.message);
  Serial.print("Target Version: "); Serial.println(packet.targetVersion);
  Serial.println("===============================");

  bool validProtocol = packet.protocolVersion == PROTOCOL_VERSION;
  bool validHub = strcmp(packet.hubId, config.targetHubId.c_str()) == 0;

  if (!validProtocol || !validHub) {
    Serial.println("[BTN] Packet ignored: invalid protocol or wrong hub.");
    return;
  }

  if (packet.messageType == MSG_ACK) {
    bool validSequence = packet.ackSequence == pendingAckSequence;
    bool validAckType = packet.ackForMessageType == pendingAckMessageType;

    if (waitingForAck && validSequence && validAckType && packet.statusCode == 0) {
      Serial.println("[BTN] Valid ACK received.");

      waitingForAck = false;
      hubConnected = true;

      setLedMode(LED_GREEN_FLASH, 2000);
    } else {
      Serial.println("[BTN] ACK validation failed or status not OK.");

      waitingForAck = false;

      if (packet.statusCode == 2) {
        setLedMode(LED_REJECT_RED_BLUE, 2000);
      } else {
        setLedMode(LED_YELLOW_BLINK, 2000);
      }
    }

    return;
  }

  if (packet.messageType == MSG_OTA_START) {
    Serial.println("[BTN] OTA START command received from Hub.");

    safeCopy(requestedOtaVersion, packet.targetVersion, sizeof(requestedOtaVersion));
    otaRequested = true;

    return;
  }

  Serial.println("[BTN] Hub packet ignored: unsupported message type.");
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  handleHubPacket(recvInfo->src_addr, incomingData, len);
}
#else
void onDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
  handleHubPacket(macAddr, incomingData, len);
}
#endif

// =====================================================
// ESP-NOW SETUP
// =====================================================

bool setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("[BTN] STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!parseMacAddress(config.targetHubMac, targetHubMacBytes)) {
    Serial.println("[BTN] ERROR: Invalid target Hub MAC.");
    setLedMode(LED_RED_SOLID);
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[BTN] ERROR: ESP-NOW init failed.");
    setLedMode(LED_RED_BLINK);
    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, targetHubMacBytes, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[BTN] ERROR: Failed to add Hub peer.");
    setLedMode(LED_RED_BLINK);
    return false;
  }

  Serial.println("[BTN] ESP-NOW ready.");
  return true;
}

// =====================================================
// RFID
// =====================================================

void setupRFID() {
  SPI.begin();
  mfrc522.PCD_Init();

  Serial.println("[BTN] RFID initialized.");
}

void checkRFID() {
  if (!hubConnected) return;
  if (waitingForAck) return;

  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  char uidString[24];
  uidString[0] = '\0';

  for (byte i = 0; i < mfrc522.uid.size; i++) {
    char byteText[4];
    snprintf(byteText, sizeof(byteText), "%02X", mfrc522.uid.uidByte[i]);
    strncat(uidString, byteText, sizeof(uidString) - strlen(uidString) - 1);
  }

  Serial.print("[BTN] RFID badge scanned: ");
  Serial.println(uidString);

  sendButtonPacket(MSG_BADGE_SCAN, uidString, 0);

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

// =====================================================
// BUTTON
// =====================================================

void checkButtonPress() {
  if (!hubConnected) return;
  if (waitingForAck) return;

  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastButtonChangeMs = millis();
    lastButtonReading = reading;
  }

  if ((millis() - lastButtonChangeMs) > BUTTON_DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        pieceCounter++;

        Serial.println();
        Serial.print("[BTN] Piece done. Counter: ");
        Serial.println(pieceCounter);

        sendButtonPacket(MSG_PIECE_DONE, nullptr, pieceCounter);
      }
    }
  }
}

// =====================================================
// OTA
// =====================================================

bool connectToWiFiForOTA() {
  if (config.wifiSsid.length() == 0) {
    Serial.println("[OTA] Wi-Fi SSID is empty.");
    return false;
  }

  Serial.println("[OTA] Switching from ESP-NOW to Wi-Fi.");

  esp_now_deinit();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());

  Serial.print("[OTA] Connecting to Wi-Fi: ");
  Serial.println(config.wifiSsid);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    setLedMode(LED_YELLOW_BLINK);
    updateLed();
    delay(20);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[OTA] Wi-Fi connected.");
    Serial.print("[OTA] IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("[OTA] Wi-Fi connection failed.");
  return false;
}

bool performDirectBinOTA(const String &url) {
  if (url.length() == 0) {
    Serial.println("[OTA] Firmware URL is empty.");
    return false;
  }

  Serial.print("[OTA] Firmware URL: ");
  Serial.println(url);

  HTTPClient http;
  WiFiClient normalClient;
  WiFiClientSecure secureClient;

  bool isHttps = url.startsWith("https://");

  if (isHttps) {
    secureClient.setInsecure();
    if (!http.begin(secureClient, url)) {
      Serial.println("[OTA] HTTPS begin failed.");
      return false;
    }
  } else {
    if (!http.begin(normalClient, url)) {
      Serial.println("[OTA] HTTP begin failed.");
      return false;
    }
  }

  http.setTimeout(20000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.print("[OTA] HTTP GET failed. Code: ");
    Serial.println(httpCode);
    http.end();
    return false;
  }

  int contentLength = http.getSize();

  Serial.print("[OTA] Content length: ");
  Serial.println(contentLength);

  bool canBegin = false;

  if (contentLength > 0) {
    canBegin = Update.begin(contentLength);
  } else {
    canBegin = Update.begin(UPDATE_SIZE_UNKNOWN);
  }

  if (!canBegin) {
    Serial.println("[OTA] Update.begin failed.");
    Update.printError(Serial);
    http.end();
    return false;
  }

  setLedMode(LED_PURPLE_BLINK);

  WiFiClient *stream = http.getStreamPtr();

  size_t written = Update.writeStream(*stream);

  Serial.print("[OTA] Written bytes: ");
  Serial.println(written);

  if (contentLength > 0 && written != (size_t)contentLength) {
    Serial.println("[OTA] Written size mismatch.");
    Update.printError(Serial);
    http.end();
    return false;
  }

  if (!Update.end()) {
    Serial.println("[OTA] Update.end failed.");
    Update.printError(Serial);
    http.end();
    return false;
  }

  if (!Update.isFinished()) {
    Serial.println("[OTA] Update not finished.");
    http.end();
    return false;
  }

  http.end();

  Serial.println("[OTA] Update successful. Rebooting.");
  setLedMode(LED_PURPLE_SOLID);
  updateLed();

  delay(1000);
  ESP.restart();

  return true;
}

void handleOtaRequest() {
  if (!otaRequested) return;

  otaRequested = false;

  Serial.println();
  Serial.println("========== OTA REQUEST ==========");

  uint16_t batteryMv = readBatteryMv();
  uint8_t batteryPercent = batteryMvToPercent(batteryMv);
  bool usb = isUsbPlugged();

  Serial.print("[OTA] Battery: ");
  Serial.print(batteryPercent);
  Serial.print("% / USB: ");
  Serial.println(usb ? "YES" : "NO");

  if (strlen(requestedOtaVersion) > 0 && strcmp(requestedOtaVersion, FIRMWARE_VERSION) == 0) {
    Serial.println("[OTA] Target version equals current version. Skipping OTA.");
    sendButtonPacket(MSG_OTA_STATUS, "ALREADY_UPDATED", 304);
    return;
  }

  if (batteryPercent < OTA_MIN_BATTERY_PERCENT && !usb) {
    Serial.println("[OTA] Rejected: battery too low and USB not plugged.");
    sendButtonPacket(MSG_OTA_STATUS, "LOW_BATTERY", 401);
    setLedMode(LED_YELLOW_BLINK, 2000);
    return;
  }

  if (config.firmwareUrl.length() == 0) {
    Serial.println("[OTA] Rejected: firmware URL is empty.");
    sendButtonPacket(MSG_OTA_STATUS, "NO_FW_URL", 400);
    setLedMode(LED_RED_BLINK, 3000);
    return;
  }

  sendButtonPacket(MSG_OTA_STATUS, "OTA_STARTING", 100);
  delay(300);

  setLedMode(LED_PURPLE_BLINK);

  if (!connectToWiFiForOTA()) {
    Serial.println("[OTA] Failed: Wi-Fi connection.");
    setLedMode(LED_OTA_FAILED, 3000);
    delay(3000);
    ESP.restart();
    return;
  }

  bool ok = performDirectBinOTA(config.firmwareUrl);

  if (!ok) {
    Serial.println("[OTA] Failed.");
    setLedMode(LED_OTA_FAILED, 3000);
    delay(3000);
    ESP.restart();
  }
}

// =====================================================
// NORMAL LOGIC
// =====================================================

void updateAckTimeout() {
  if (!waitingForAck) return;

  if (millis() - lastAckWaitStartMs > ACK_TIMEOUT_MS) {
    Serial.println("[BTN] ACK timeout.");

    waitingForAck = false;

    if (pendingAckMessageType == MSG_HELLO) {
      hubConnected = false;
      setLedMode(LED_YELLOW_BLINK);
    } else {
      setLedMode(LED_YELLOW_BLINK, 2000);
    }
  }
}

void updateConnectionLogic() {
  unsigned long now = millis();

  if (!hubConnected) {
    if (currentLedMode != LED_GREEN_FLASH) {
      setLedMode(LED_YELLOW_BLINK);
    }

    if (!waitingForAck && now - lastHelloAttemptMs >= HELLO_RETRY_INTERVAL_MS) {
      lastHelloAttemptMs = now;
      sendButtonPacket(MSG_HELLO, nullptr, 0);
    }

    return;
  }

  if (currentLedMode != LED_GREEN_FLASH &&
      currentLedMode != LED_YELLOW_BLINK &&
      currentLedMode != LED_REJECT_RED_BLUE) {
    setLedMode(LED_GREEN_SOLID);
  }
}

void updatePeriodicMessages() {
  if (!hubConnected) return;
  if (waitingForAck) return;

  unsigned long now = millis();

  if (now - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = now;
    sendButtonPacket(MSG_HEARTBEAT, nullptr, 0);
  }

  if (now - lastBatteryStatusMs >= BATTERY_STATUS_INTERVAL_MS) {
    lastBatteryStatusMs = now;

    uint16_t batteryMv = readBatteryMv();
    sendButtonPacket(MSG_BATTERY_STATUS, nullptr, batteryMv);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);

  pinMode(USB_PIN, INPUT);

  analogReadResolution(12);

  setLED(false, false, false);
  setLedMode(LED_BOOT_BLUE_BLINK);

  bootId = esp_random();
  if (bootId == 0) bootId = millis();

  Serial.println();
  Serial.println("=================================================");
  Serial.println("ESS TECHNOLOGIES - SMART BUTTON FIRMWARE");
  Serial.println("=================================================");
  Serial.print("Firmware Version: ");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Boot ID: ");
  Serial.println(bootId);
  Serial.println("=================================================");

  loadConfigFromNVS();

  if (shouldEnterSetupPortal()) {
    startConfigPortal();
  }

  setupRFID();

  if (!setupEspNow()) {
    Serial.println("[BOOT] ESP-NOW setup failed.");
    setLedMode(LED_RED_BLINK);
  }

  lastHelloAttemptMs = millis() - HELLO_RETRY_INTERVAL_MS;
  lastHeartbeatMs = millis();
  lastBatteryStatusMs = millis();

  Serial.println("[BOOT] Normal mode started.");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  updateLed();

  handleOtaRequest();

  updateAckTimeout();
  updateConnectionLogic();

  checkRFID();
  checkButtonPress();

  updatePeriodicMessages();
}
