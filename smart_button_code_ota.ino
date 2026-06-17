#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>

#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

#include <SPI.h>
#include <MFRC522.h>

// =====================================================
// FIRMWARE VERSION
// Bump this string after building and pushing new .bin to GitHub.
// Also update version.txt in the repo root to match.
// =====================================================

#define BTN_FIRMWARE_VERSION "1.2.0"

// =====================================================
// BOOTSTRAP CONFIG
// CHANGE THIS ONLY FOR FIRST MANUAL USB UPLOAD
// =====================================================

// #define BOOTSTRAP_DEVICE_NAME "BTN-L1N1"
// #define BOOTSTRAP_HUB_NAME    "HUB-L1"
// #define BOOTSTRAP_HUB_MAC     "F4:65:0B:49:AA:64"

#define BOOTSTRAP_DEVICE_NAME "BTN-L1N2"
#define BOOTSTRAP_HUB_NAME    "HUB-L1"
#define BOOTSTRAP_HUB_MAC     "F4:65:0B:49:AA:64"

// #define BOOTSTRAP_DEVICE_NAME "BTN-L2N1"
// #define BOOTSTRAP_HUB_NAME    "HUB-L2"
// #define BOOTSTRAP_HUB_MAC     "EC:E3:34:9B:08:F0"

// #define BOOTSTRAP_DEVICE_NAME "BTN-L2N2"
// #define BOOTSTRAP_HUB_NAME    "HUB-L2"
// #define BOOTSTRAP_HUB_MAC     "EC:E3:34:9B:08:F0"



// true  = first manual upload to write identity into NVS
// false = OTA universal firmware, identity will NOT change
#define BOOTSTRAP_FORCE_OVERWRITE false

// =====================================================
// OTA URL
// =====================================================

#define BUTTON_BIN_URL "https://raw.githubusercontent.com/ismailoviic/smart_button_code_ota/main/build/esp32dev/firmware.bin"

// =====================================================
// PINS
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
// SETTINGS
// =====================================================

#define AP_PASSWORD "12345678"
#define ESPNOW_CHANNEL 8  // fixed channel for the button's own local setup AP only

// The Hub's actual ESP-NOW channel follows whatever Wi-Fi router it's
// connected to (ESP32 has one radio — STA and ESP-NOW share it), so it can
// change at any time. The button doesn't join any router, so instead of
// assuming a fixed channel, it hops through all of them looking for the Hub.
#define CHANNEL_SCAN_MIN        1
#define CHANNEL_SCAN_MAX        13
#define CHANNEL_DWELL_MS        300   // time to wait for an ACK on each channel
#define BOOT_DISCOVERY_TIMEOUT_MS   30000  // full scan budget before falling back to portal
#define RECONNECT_SCAN_TIMEOUT_MS   8000   // scan budget when a live connection drops

IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

Preferences prefs;
WebServer server(80);
DNSServer dnsServer;
MFRC522 mfrc522(SS_PIN, RST_PIN);

const byte DNS_PORT = 53;

// =====================================================
// STORED CONFIG
// =====================================================

String deviceName;
String hubName;
String hubMacText;
String wifiSSID;
String wifiPassword;

uint8_t hubMac[6];

bool hubConnected = false;
uint8_t currentChannel = ESPNOW_CHANNEL;

unsigned long lastHelloMs = 0;
unsigned long packetCounter = 0;
unsigned long lastButtonMs = 0;
unsigned long lastAckMs = 0;
unsigned long lastPingMs = 0;

#define PING_INTERVAL_MS      8000   // periodic keep-alive HELLO sent to hub
#define CONNECTION_TIMEOUT_MS 20000  // no ACK within this window -> considered disconnected

// =====================================================
// SIMPLE ESP-NOW PROTOCOL
// =====================================================

#define MSG_HELLO        1
#define MSG_ACK          2
#define MSG_BUTTON_PRESS 3
#define MSG_RFID_SCAN    4

typedef struct __attribute__((packed)) {
  uint8_t type;
  char from[16];
  char to[16];
  uint32_t counter;
  char text[32];
} SimplePacket;

// =====================================================
// LED
// =====================================================

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_RED_PIN, r ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, g ? HIGH : LOW);
  digitalWrite(LED_BLUE_PIN, b ? HIGH : LOW);
}

void ledRed()    { setLED(true, false, false); }
void ledGreen()  { setLED(false, true, false); }
void ledYellow() { setLED(true, true, false); }
void ledBlue()   { setLED(false, false, true); }
void ledPurple() { setLED(true, false, true); }
void ledCyan()   { setLED(false, true, true); }  // OTA in progress: green + blue

void updateConnectionLED() {
  if (hubConnected) ledGreen(); else ledRed();
}

void blinkGreen() {
  Serial.println("[LED] Blink GREEN");
  ledGreen();
  delay(150);
  setLED(false, false, false);
  delay(100);
  ledGreen();
  delay(150);
  ledGreen();
}

void blinkBlue() {
  Serial.println("[LED] Blink BLUE");
  ledBlue();
  delay(200);
  setLED(false, false, false);
  delay(100);
  ledBlue();
  delay(200);
  ledGreen();
}

void blinkPurpleDuringOTA() {
  ledCyan();
  delay(150);
  setLED(false, false, false);
  delay(150);
}

// =====================================================
// BATTERY
// =====================================================

uint16_t readBatteryMv() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(BATT_PIN);
  return (uint16_t)((sum / 8) * 3300UL * 2 / 4095);
}

uint8_t batteryPercent(uint16_t mv) {
  if (mv >= 4200) return 100;
  if (mv <= 3000) return 0;
  return (uint8_t)((mv - 3000UL) * 100 / 1200);
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

bool parseMac(String macText, uint8_t *mac) {
  int values[6];

  if (sscanf(
        macText.c_str(),
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

// =====================================================
// NVS CONFIG
// =====================================================

void saveBootstrapIdentityIfNeeded() {
  prefs.begin("btn_cfg", false);

  bool initialized = prefs.getBool("initialized", false);

  if (!initialized || BOOTSTRAP_FORCE_OVERWRITE) {
    Serial.println("[NVS] Writing bootstrap identity.");

    prefs.putString("device", BOOTSTRAP_DEVICE_NAME);
    prefs.putString("hub", BOOTSTRAP_HUB_NAME);
    prefs.putString("hubmac", BOOTSTRAP_HUB_MAC);

    if (!initialized) {
      prefs.putString("ssid", "");
      prefs.putString("pass", "");
    }

    prefs.putBool("initialized", true);
  }

  prefs.end();
}

void loadConfig() {
  saveBootstrapIdentityIfNeeded();

  prefs.begin("btn_cfg", true);

  deviceName = prefs.getString("device", BOOTSTRAP_DEVICE_NAME);
  hubName = prefs.getString("hub", BOOTSTRAP_HUB_NAME);
  hubMacText = prefs.getString("hubmac", BOOTSTRAP_HUB_MAC);
  wifiSSID = prefs.getString("ssid", "");
  wifiPassword = prefs.getString("pass", "");

  prefs.end();

  hubMacText.toUpperCase();

  Serial.println();
  Serial.println("========== LOADED BUTTON CONFIG ==========");
  Serial.print("Device Name: "); Serial.println(deviceName);
  Serial.print("Hub Name: "); Serial.println(hubName);
  Serial.print("Hub MAC: "); Serial.println(hubMacText);
  Serial.print("Wi-Fi SSID: "); Serial.println(wifiSSID);
  Serial.println("==========================================");

  if (!parseMac(hubMacText, hubMac)) {
    Serial.println("[NVS] ERROR: Invalid Hub MAC.");
  }
}

void saveConfig(String newDevice, String newHub, String newHubMac, String ssid, String pass) {
  newHubMac.toUpperCase();

  prefs.begin("btn_cfg", false);

  prefs.putString("device", newDevice);
  prefs.putString("hub", newHub);
  prefs.putString("hubmac", newHubMac);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putBool("initialized", true);

  prefs.end();

  Serial.println("[NVS] Config saved.");

  loadConfig();
}

// =====================================================
// OTA
// =====================================================

bool connectWiFiForOTA() {
  if (wifiSSID.length() == 0) {
    Serial.println("[OTA] No Wi-Fi SSID saved.");
    return false;
  }

  Serial.println("[OTA] Connecting to Wi-Fi...");
  Serial.print("[OTA] SSID: ");
  Serial.println(wifiSSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    blinkPurpleDuringOTA();
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[OTA] Wi-Fi connected.");
    Serial.print("[OTA] IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("[OTA] Wi-Fi failed.");
  return false;
}

bool performOTA() {
  Serial.println();
  Serial.println("========== BUTTON OTA START ==========");
  Serial.print("[OTA] URL: ");
  Serial.println(BUTTON_BIN_URL);

  HTTPClient http;
  WiFiClientSecure client;
  client.setInsecure();

  if (!http.begin(client, BUTTON_BIN_URL)) {
    Serial.println("[OTA] HTTP begin failed.");
    return false;
  }

  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(30000);
  http.addHeader("Cache-Control", "no-cache");

  int httpCode = http.GET();

  Serial.print("[OTA] HTTP code: ");
  Serial.println(httpCode);

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("[OTA] Download failed.");
    http.end();
    return false;
  }

  int contentLength = http.getSize();

  Serial.print("[OTA] Content length: ");
  Serial.println(contentLength);

  if (!Update.begin(contentLength > 0 ? contentLength : UPDATE_SIZE_UNKNOWN)) {
    Serial.println("[OTA] Update.begin failed.");
    Update.printError(Serial);
    http.end();
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();

  size_t written = Update.writeStream(*stream);

  Serial.print("[OTA] Written bytes: ");
  Serial.println(written);

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

  Serial.println("[OTA] SUCCESS. Restarting.");
  ledCyan();
  delay(1000);
  ESP.restart();

  return true;
}

void startOTAFromPortal() {
  Serial.println("[PORTAL] OTA requested.");

  server.send(200, "text/html", "<h2>OTA started</h2><p>Device is updating from GitHub.</p>");

  delay(1000);

  esp_now_deinit();

  if (!connectWiFiForOTA()) {
    Serial.println("[OTA] Wi-Fi failed. Restarting.");
    delay(3000);
    ESP.restart();
  }

  if (!performOTA()) {
    Serial.println("[OTA] Update failed. Restarting.");
    delay(3000);
    ESP.restart();
  }
}

// =====================================================
// WEB PORTAL
// =====================================================

String htmlPage() {
  String html = "";

  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Button OTA Setup</title>";
  html += "<style>";
  html += "body{font-family:Arial;background:#f4f4f4;padding:20px;}";
  html += ".card{background:white;padding:20px;border-radius:12px;max-width:450px;margin:auto;box-shadow:0 2px 8px rgba(0,0,0,.15);}";
  html += "input{width:100%;padding:12px;margin:8px 0;box-sizing:border-box;}";
  html += "button{width:100%;padding:14px;margin-top:10px;background:#111;color:white;border:0;border-radius:6px;font-size:16px;}";
  html += ".info{background:#eee;padding:10px;border-radius:6px;margin-bottom:10px;font-size:14px;}";
  html += "</style></head><body><div class='card'>";

  html += "<h2>Smart Button OTA Setup</h2>";

  uint16_t battMv  = readBatteryMv();
  uint8_t  battPct = batteryPercent(battMv);

  html += "<div class='info'>";
  html += "<b>Device:</b> " + deviceName;
  html += "<br><b>Hub:</b> " + hubName;
  html += "<br><b>Hub MAC:</b> " + hubMacText;
  html += "<br><b>Battery:</b> " + String(battMv) + " mV (" + String(battPct) + "%)";
  html += "<br><b>Setup:</b> http://192.168.4.1";
  html += "<br><b>OTA URL:</b><br>";
  html += BUTTON_BIN_URL;
  html += "</div>";

  html += "<form action='/save' method='POST'>";

  html += "<label>Device Name</label>";
  html += "<input name='device' value='" + deviceName + "'>";

  html += "<label>Hub Name</label>";
  html += "<input name='hub' value='" + hubName + "'>";

  html += "<label>Hub MAC</label>";
  html += "<input name='hubmac' value='" + hubMacText + "'>";

  html += "<label>Wi-Fi SSID</label>";
  html += "<input name='ssid' value='" + wifiSSID + "'>";

  html += "<label>Wi-Fi Password</label>";
  html += "<input name='pass' type='password' value='" + wifiPassword + "'>";

  html += "<button type='submit'>Save Config</button>";
  html += "</form>";

  html += "<form action='/update' method='POST'>";
  html += "<button type='submit'>Update From GitHub Now</button>";
  html += "</form>";

  html += "</div></body></html>";

  return html;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleSave() {
  saveConfig(
    server.arg("device"),
    server.arg("hub"),
    server.arg("hubmac"),
    server.arg("ssid"),
    server.arg("pass")
  );

  server.send(200, "text/html", "<h2>Saved</h2><p>Config saved.</p><p>If you changed identity, restart device.</p><a href='/'>Back</a>");
}

void handleCaptive() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

void startPortal() {
  Serial.println();
  Serial.println("========== BUTTON PORTAL START ==========");
  Serial.print("[PORTAL] SSID: ");
  Serial.println(deviceName);
  Serial.print("[PORTAL] Password: ");
  Serial.println(AP_PASSWORD);
  Serial.println("[PORTAL] URL: http://192.168.4.1");
  Serial.println("=========================================");

  ledYellow();

  esp_now_deinit();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(deviceName.c_str(), AP_PASSWORD, ESPNOW_CHANNEL);

  dnsServer.start(DNS_PORT, "*", apIP);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/update", HTTP_POST, startOTAFromPortal);

  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.on("/fwlink", HTTP_GET, handleRoot);

  server.onNotFound(handleCaptive);

  server.begin();

  while (true) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(2);
  }
}

// =====================================================
// ESP-NOW
// =====================================================

void sendPacket(uint8_t type, const char *text) {
  SimplePacket packet;

  packet.type = type;
  safeCopy(packet.from, deviceName.c_str(), sizeof(packet.from));
  safeCopy(packet.to, hubName.c_str(), sizeof(packet.to));
  packet.counter = ++packetCounter;
  safeCopy(packet.text, text, sizeof(packet.text));

  Serial.println();
  Serial.println("========== BUTTON SEND ESP-NOW ==========");
  Serial.print("Type: "); Serial.println(type);
  Serial.print("From: "); Serial.println(packet.from);
  Serial.print("To: "); Serial.println(packet.to);
  Serial.print("Counter: "); Serial.println(packet.counter);
  Serial.print("Text: "); Serial.println(packet.text);
  Serial.print("Hub MAC: "); Serial.println(macToString(hubMac));
  Serial.println("=========================================");

  esp_err_t result = esp_now_send(hubMac, (uint8_t *)&packet, sizeof(packet));

  if (result == ESP_OK) {
    Serial.println("[ESP-NOW] Send request OK.");
  } else {
    Serial.print("[ESP-NOW] Send failed. Code: ");
    Serial.println(result);
  }
}

void sendHello() {
  char text[32];
  uint16_t mv = readBatteryMv();
  snprintf(text, sizeof(text), "HELLO:%u", mv);
  sendPacket(MSG_HELLO, text);
}

static void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Send callback: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "LOW LEVEL SUCCESS" : "LOW LEVEL FAIL");
}

void handleReceivedData(const uint8_t *senderMac, const uint8_t *incomingData, int len) {
  if (len != sizeof(SimplePacket)) {
    Serial.println("[ESP-NOW] Invalid packet size.");
    return;
  }

  SimplePacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  Serial.println();
  Serial.println("========== BUTTON RECEIVED ESP-NOW ==========");
  Serial.print("From MAC: "); Serial.println(macToString(senderMac));
  Serial.print("Type: "); Serial.println(packet.type);
  Serial.print("From: "); Serial.println(packet.from);
  Serial.print("To: "); Serial.println(packet.to);
  Serial.print("Text: "); Serial.println(packet.text);
  Serial.println("=============================================");

  if (strcmp(packet.to, deviceName.c_str()) != 0) return;

  if (packet.type == MSG_ACK) {
    if (strcmp(packet.text, "OTA_START") == 0) {
      Serial.println("[ESP-NOW] OTA_START received from Hub. Starting update...");
      ledCyan();
      delay(500);
      esp_now_deinit();
      if (connectWiFiForOTA()) {
        performOTA();
      } else {
        Serial.println("[OTA] Wi-Fi failed after OTA_START. Restarting.");
        delay(2000);
        ESP.restart();
      }
    } else {
      Serial.println("[ESP-NOW] ACK received from Hub.");
      hubConnected = true;
      lastAckMs = millis();
      updateConnectionLED();
    }
  }
}

static void onDataRecv(const uint8_t *macAddr, const uint8_t *incomingData, int len) {
  handleReceivedData(macAddr, incomingData, len);
}

void setRadioChannel(uint8_t ch) {
  if (ch < CHANNEL_SCAN_MIN || ch > CHANNEL_SCAN_MAX) return;
  currentChannel = ch;
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

bool setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  esp_wifi_set_ps(WIFI_PS_NONE);
  setRadioChannel(currentChannel);

  Serial.print("[ESP-NOW] Button STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!parseMac(hubMacText, hubMac)) {
    Serial.println("[ESP-NOW] Invalid Hub MAC.");
    return false;
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed.");
    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, hubMac, 6);
  peerInfo.channel = 0;  // use whatever channel the radio is currently on
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] Failed to add Hub peer.");
    return false;
  }

  Serial.println("[ESP-NOW] Ready.");
  return true;
}

// Dwell on one channel for dwellMs, pinging the Hub, to see if it answers there.
bool tryChannel(uint8_t ch, unsigned long dwellMs) {
  setRadioChannel(ch);

  unsigned long start = millis();
  while (millis() - start < dwellMs) {
    if (millis() - lastHelloMs >= 120) {
      lastHelloMs = millis();
      sendHello();
    }
    delay(10);
    if (hubConnected) return true;
  }
  return hubConnected;
}

// Hops across all Wi-Fi channels looking for the Hub's current ESP-NOW
// channel (the Hub's channel follows whatever router it's joined, so it's
// not fixed). Stops as soon as an ACK is received, or after totalTimeoutMs.
void discoverHub(unsigned long totalTimeoutMs) {
  Serial.println();
  Serial.println("========== SCANNING CHANNELS FOR HUB ==========");

  ledRed();

  unsigned long start = millis();

  while (millis() - start < totalTimeoutMs && !hubConnected) {
    for (uint8_t ch = CHANNEL_SCAN_MIN; ch <= CHANNEL_SCAN_MAX; ch++) {
      if (tryChannel(ch, CHANNEL_DWELL_MS)) break;
      if (millis() - start >= totalTimeoutMs) break;
    }
  }

  if (hubConnected) {
    Serial.print("[ESP-NOW] Hub found on channel ");
    Serial.println(currentChannel);
    ledGreen();
  } else {
    Serial.println("[ESP-NOW] Hub not found.");
  }
}

// =====================================================
// RFID + BUTTON
// =====================================================

void setupRFID() {
  SPI.begin();
  mfrc522.PCD_Init();

  Serial.println("[RFID] MFRC522 initialized.");
}

void checkRFID() {
  if (!hubConnected) return;

  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  char uid[32];
  uid[0] = '\0';

  for (byte i = 0; i < mfrc522.uid.size; i++) {
    char part[4];
    snprintf(part, sizeof(part), "%02X", mfrc522.uid.uidByte[i]);
    strncat(uid, part, sizeof(uid) - strlen(uid) - 1);
  }

  Serial.println();
  Serial.print("[RFID] UID scanned: ");
  Serial.println(uid);

  blinkBlue();
  sendPacket(MSG_RFID_SCAN, uid);

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void checkButton() {
  if (!hubConnected) return;

  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButtonMs > 700) {
    lastButtonMs = millis();

    Serial.println();
    Serial.println("[BUTTON] Button pressed.");

    blinkGreen();
    sendPacket(MSG_BUTTON_PRESS, "BUTTON_PRESS");
  }
}

// =====================================================
// SETUP + LOOP
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

  Serial.println();
  Serial.println("=========================================");
  Serial.println("ESS SMART BUTTON - OTA SAFE IDENTITY");
  Serial.println("=========================================");
  Serial.print("BOOTSTRAP_FORCE_OVERWRITE: ");
  Serial.println(BOOTSTRAP_FORCE_OVERWRITE ? "true" : "false");
  Serial.print("OTA URL: ");
  Serial.println(BUTTON_BIN_URL);
  Serial.println("=========================================");

  loadConfig();

  setupRFID();

  if (!setupEspNow()) {
    Serial.println("[BOOT] ESP-NOW failed. Starting portal.");
    ledYellow();
    startPortal();
  }

  discoverHub(BOOT_DISCOVERY_TIMEOUT_MS);

  if (!hubConnected) {
    Serial.println("[BOOT] Hub not found. LED YELLOW. Starting portal.");
    ledYellow();
    startPortal();
  }
}

void checkConnectionPing() {
  if (millis() - lastPingMs >= PING_INTERVAL_MS) {
    lastPingMs = millis();
    sendHello();
  }
  if (hubConnected && millis() - lastAckMs > CONNECTION_TIMEOUT_MS) {
    Serial.println("[ESP-NOW] Hub ACK timed out. Marking disconnected.");
    hubConnected = false;
    // Hub may have rebooted onto a different Wi-Fi channel — re-scan instead
    // of retrying forever on a channel the Hub may no longer be using.
    discoverHub(RECONNECT_SCAN_TIMEOUT_MS);
  }
}

void loop() {
  updateConnectionLED();
  checkConnectionPing();
  checkButton();
  checkRFID();
}