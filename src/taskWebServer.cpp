#include <list>
#include "taskWebServer.h"
#include "preference_storage.h"
#include "syslog_log.h"
#include "PSRAMJsonDocument.h"
#include <time.h>
#include <ArduinoJson.h>

/**
 * @see board_build.embed_txtfiles in platformio.ini
 */
extern const char web_index_html[] asm("_binary_data_embed_index_html_out_start");
extern const char web_index_html_end[] asm("_binary_data_embed_index_html_out_end");
extern const char web_style_css[] asm("_binary_data_embed_style_css_out_start");
extern const char web_style_css_end[] asm("_binary_data_embed_style_css_out_end");
extern const char web_js_js[] asm("_binary_data_embed_js_js_out_start");
extern const char web_js_js_end[] asm("_binary_data_embed_js_js_out_end");

// Variable needed to send beacon from html page
extern bool manBeacon;

// Variable to show AP status
extern bool apEnabled;
extern bool apConnected;
extern String infoApName;
extern String infoApPass;
extern String infoApAddr;

// For APRS-IS connection
extern String to_aprsis_data;
extern boolean aprsis_enabled;
extern String aprsis_host;
extern uint16_t aprsis_port;
extern String aprsis_filter;
extern String aprsis_callsign;
extern String aprsis_password;
extern boolean aprsis_data_allow_inet_to_rf;

extern double lora_freq_rx_curr;
extern boolean lora_tx_enabled;


QueueHandle_t webListReceivedQueue = nullptr;
std::list <tReceivedPacketData*> receivedPackets;
const int MAX_RECEIVED_LIST_SIZE = 50;

String apSSID = "";
String apPassword;
String defApPassword = "xxxxxxxxxx";

// needed for aprsis igate functionality
String aprsis_status = "Disconnected";
// aprsis 3rd party traffic encoding
String generate_third_party_packet(String, String);
#ifdef KISS_PROTOCOL
extern void sendToTNC(const String &);
#endif
extern uint8_t txPower;
extern double lora_freq;
extern ulong lora_speed;
extern void loraSend(byte, float, ulong, const String &);


WebServer server(80);
#ifdef KISS_PROTOCOL
  WiFiServer tncServer(NETWORK_TNC_PORT);
#endif
WiFiServer gpsServer(NETWORK_GPS_PORT);

#ifdef ENABLE_SYSLOG
  // A UDP instance to let us send and receive packets over UDP
  WiFiUDP udpClient;

  // Create a new empty syslog instance
  Syslog syslog(udpClient, SYSLOG_PROTO_IETF);
#endif

#ifdef T_BEAM_V1_0
  #include <axp20x.h>
  extern AXP20X_Class axp;
#endif


void sendCacheHeader() { server.sendHeader("Cache-Control", "max-age=3600"); }
void sendGzipHeader() { server.sendHeader("Content-Encoding", "gzip"); }

String jsonEscape(String s){
    s.replace("\\", "\\\\");
    s.replace("\"", "\\\"");
    s.replace("\x7f", "\\\x7f");
    for(char i = 0; i < 0x1f; i++){
        s.replace(String(i), "\\" + String((char)i));
    }
  return s;
}

String jsonLineFromPreferenceString(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":\"" + jsonEscape(preferences.getString(preferenceName)) + (last ?  + R"(")" :  + R"(",)");
}
String jsonLineFromPreferenceBool(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":" + (preferences.getBool(preferenceName) ? "true" : "false") + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromPreferenceInt(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":" + (preferences.getInt(preferenceName)) + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromPreferenceDouble(const char *preferenceName, bool last=false){
    return String("\"") + preferenceName + "\":" + String(preferences.getDouble(preferenceName),3) + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromString(const char *name, const char *value, bool last=false){
  return String("\"") + name + "\":\"" + jsonEscape(value) + "\"" + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromInt(const char *name, const int value, bool last=false){
  return String("\"") + name + "\":" + String(value) + (last ?  + R"()" :  + R"(,)");
}

void handle_NotFound(){
  sendCacheHeader();
  server.send(404, "text/plain", "Not found");
}

void handle_Index() {
  sendGzipHeader();
  server.send_P(200, "text/html", web_index_html, web_index_html_end - web_index_html);
}

void handle_Style() {
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/css", web_style_css, web_style_css_end - web_style_css);
}

void handle_Js() {
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/javascript", web_js_js, web_js_js_end-web_js_js);
}

void handle_ScanWifi() {
  String listResponse = R"(<label for="networks_found_list">Networks found:</label><select class="u-full-width" id="networks_found_list">)";
  int n = WiFi.scanNetworks();
  listResponse += "<option value=\"\">Select Network</option>";

  for (int i = 0; i < n; ++i) {
    listResponse += "<option value=\""+WiFi.SSID(i)+"\">" + WiFi.SSID(i) + "</option>";
  }
  listResponse += "</select>";
  server.send(200,"text/html", listResponse);
}

void handle_SaveWifiCfg() {
  if (!server.hasArg(PREF_WIFI_SSID) || !server.hasArg(PREF_WIFI_PASSWORD) || !server.hasArg(PREF_AP_PASSWORD)){
    server.send(500, "text/plain", "Invalid request, make sure all fields are set");
  }

  if (!server.arg(PREF_WIFI_SSID).length()){
    server.send(403, "text/plain", "Empty SSID");
  } else {
    // Update SSID
    preferences.putString(PREF_WIFI_SSID, server.arg(PREF_WIFI_SSID));
    Serial.println("Updated SSID: " + server.arg(PREF_WIFI_SSID));
  }

  if (server.arg(PREF_WIFI_PASSWORD)!="*" && server.arg(PREF_WIFI_PASSWORD).length()>0 && server.arg(PREF_WIFI_PASSWORD).length()<8){
    server.send(403, "text/plain", "WiFi Password must be minimum 8 character");
  } else {
    if (server.arg(PREF_WIFI_PASSWORD)!="*") {
      // Update WiFi password
      preferences.putString(PREF_WIFI_PASSWORD, server.arg(PREF_WIFI_PASSWORD));
      Serial.println("Updated WiFi PASS: " + server.arg(PREF_WIFI_PASSWORD));
    }
  }

  if (server.arg(PREF_AP_PASSWORD)!="*" && server.arg(PREF_AP_PASSWORD).length()<8){
    server.send(403, "text/plain", "AP Password must be minimum 8 character");
  } else {
    if (server.arg(PREF_AP_PASSWORD)!="*") {
      // Update AP password
      preferences.putString(PREF_AP_PASSWORD, server.arg(PREF_AP_PASSWORD));
      Serial.println("Updated AP PASS: " + server.arg(PREF_AP_PASSWORD));
    }
  }

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
}

void handle_Reboot() {
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  server.close();
  ESP.restart();
}

void handle_Beacon() {
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  manBeacon=true;
}

void handle_Shutdown() {
  #ifdef T_BEAM_V1_0
    server.send(200,"text/html", "Shutdown");
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.shutdown();
  #else
    server.send(404,"text/html", "Not supported");
  #endif
}

void handle_Restore() {
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  preferences.clear();
  preferences.end();
  ESP.restart();
}

void handle_Cfg() {
  String jsonData = "{";
  jsonData += String("\"") + PREF_WIFI_PASSWORD + "\": \"" + jsonEscape((preferences.getString(PREF_WIFI_PASSWORD).isEmpty() ? String("") : "*")) + R"(",)";
  jsonData += String("\"") + PREF_AP_PASSWORD + "\": \"" + jsonEscape((preferences.getString(PREF_AP_PASSWORD).isEmpty() ? String("") : "*")) + R"(",)";
  jsonData += jsonLineFromPreferenceString(PREF_WIFI_SSID);
  jsonData += jsonLineFromPreferenceDouble(PREF_LORA_FREQ_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_SPEED_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_LORA_TX_ENABLE);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_TX_POWER);
  jsonData += jsonLineFromPreferenceBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_DIGIPEATING_MODE_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_TX_BEACON_AND_KISS_ON_FREQUENCIES_PRESET);
  jsonData += jsonLineFromPreferenceDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_SPEED_CROSSDIGI_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_TX_POWER_CROSSDIGI_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_LORA_RX_ON_FREQUENCIES_PRESET);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_CALLSIGN);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_RELAY_PATH);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_SYMBOL_TABLE);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_SYMBOL);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_COMMENT);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_LATITUDE_PRESET);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_LONGITUDE_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MIN_INTERVAL_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MAX_INTERVAL_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MIN_SPEED_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_MAX_SPEED_PRESET);
  jsonData += jsonLineFromPreferenceDouble(PREF_APRS_SB_ANGLE_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_TURN_SLOPE_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_SB_TURN_TIME_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_BATTERY);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_FIXED_BEACON_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_ALTITUDE);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_GPS_EN);
  jsonData += jsonLineFromPreferenceBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS);
  jsonData += jsonLineFromPreferenceBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS);
  jsonData += jsonLineFromPreferenceBool(PREF_ENABLE_TNC_SELF_TELEMETRY);
  jsonData += jsonLineFromPreferenceInt(PREF_TNC_SELF_TELEMETRY_INTERVAL);
  jsonData += jsonLineFromPreferenceInt(PREF_TNC_SELF_TELEMETRY_MIC);
  jsonData += jsonLineFromPreferenceString(PREF_TNC_SELF_TELEMETRY_PATH);
  jsonData += jsonLineFromPreferenceBool(PREF_DEV_OL_EN);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_CMT);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_COMMENT_RATELIMIT_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_DEV_BT_EN);
  jsonData += jsonLineFromPreferenceInt(PREF_DEV_SHOW_RX_TIME);
  jsonData += jsonLineFromPreferenceBool(PREF_DEV_AUTO_SHUT);
  jsonData += jsonLineFromPreferenceInt(PREF_DEV_AUTO_SHUT_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_DEV_SHOW_OLED_TIME);
  jsonData += jsonLineFromPreferenceBool(PREF_APRSIS_EN);
  jsonData += jsonLineFromPreferenceString(PREF_APRSIS_SERVER_NAME);
  jsonData += jsonLineFromPreferenceInt(PREF_APRSIS_SERVER_PORT);
  jsonData += jsonLineFromPreferenceString(PREF_APRSIS_FILTER);
  jsonData += jsonLineFromPreferenceString(PREF_APRSIS_CALLSIGN);
  jsonData += jsonLineFromPreferenceString(PREF_APRSIS_PASSWORD);
  jsonData += jsonLineFromPreferenceBool(PREF_APRSIS_ALLOW_INET_TO_RF);
  jsonData += jsonLineFromInt("lora_freq_rx_curr", (unsigned long ) (lora_freq_rx_curr*1000L));
  //jsonData += jsonLineFromPreferenceDouble("lora_freq_rx_curr", lora_freq_rx_curr);
  jsonData += jsonLineFromString("aprsis_status", aprsis_status.c_str());
  jsonData += jsonLineFromInt("FreeHeap", ESP.getFreeHeap());
  jsonData += jsonLineFromInt("HeapSize", ESP.getHeapSize());
  jsonData += jsonLineFromInt("FreeSketchSpace", ESP.getFreeSketchSpace());
  jsonData += jsonLineFromInt("PSRAMSize", ESP.getPsramSize());
  jsonData += jsonLineFromInt("PSRAMFree", ESP.getFreePsram(), true);

  jsonData += "}";
  server.send(200,"application/json", jsonData);
}

void handle_ReceivedList() {
  //PSRAMJsonDocument doc(MAX_RECEIVED_LIST_SIZE * 1000);
  DynamicJsonDocument doc(MAX_RECEIVED_LIST_SIZE * 500);
  JsonObject root = doc.to<JsonObject>();
  auto received = root.createNestedArray("received");
  for (auto element: receivedPackets){
    char buf[64];
    strftime(buf, 64, "%Y-%m-%d %H:%M:%S", &element->rxTime);
    auto packet_data = received.createNestedObject();
    packet_data["time"] = String(buf);
    packet_data["packet"] = element->packet->c_str();
    packet_data["rssi"] = element->RSSI;
    packet_data["snr"] = element->SNR;
  }

  server.send(200,"application/json", doc.as<String>());
}

void handle_SaveAPRSCfg() {
  // LoRa settings
  if (server.hasArg(PREF_LORA_FREQ_PRESET)){
    preferences.putDouble(PREF_LORA_FREQ_PRESET, server.arg(PREF_LORA_FREQ_PRESET).toDouble());
    Serial.printf("FREQ saved:\t%f\n", server.arg(PREF_LORA_FREQ_PRESET).toDouble());
  }
  if (server.hasArg(PREF_LORA_SPEED_PRESET)){
    preferences.putInt(PREF_LORA_SPEED_PRESET, server.arg(PREF_LORA_SPEED_PRESET).toInt());
  }
  preferences.putBool(PREF_LORA_TX_ENABLE, server.hasArg(PREF_LORA_TX_ENABLE));
  if (server.hasArg(PREF_LORA_TX_POWER)) {
    preferences.putInt(PREF_LORA_TX_POWER, server.arg(PREF_LORA_TX_POWER).toInt());
  }
  preferences.putBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET, server.hasArg(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET));
  if (server.hasArg(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET)){
    preferences.putInt(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET, server.arg(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET).toInt());
  }
  preferences.putBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET, server.hasArg(PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET));
  if (server.hasArg(PREF_APRS_DIGIPEATING_MODE_PRESET)){
    preferences.putInt(PREF_APRS_DIGIPEATING_MODE_PRESET, server.arg(PREF_APRS_DIGIPEATING_MODE_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET)){
    preferences.putInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET, server.arg(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET).toInt());
  }
  if (server.hasArg(PREF_LORA_TX_BEACON_AND_KISS_ON_FREQUENCIES_PRESET)) {
    preferences.putInt(PREF_LORA_TX_BEACON_AND_KISS_ON_FREQUENCIES_PRESET, server.arg(PREF_LORA_TX_BEACON_AND_KISS_ON_FREQUENCIES_PRESET).toInt());
  }
  if (server.hasArg(PREF_LORA_FREQ_CROSSDIGI_PRESET)){
    preferences.putDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET, server.arg(PREF_LORA_FREQ_CROSSDIGI_PRESET).toDouble());
    Serial.printf("FREQ crossdigi saved:\t%f\n", server.arg(PREF_LORA_FREQ_CROSSDIGI_PRESET).toDouble());
  }
  if (server.hasArg(PREF_LORA_SPEED_CROSSDIGI_PRESET)){
    preferences.putInt(PREF_LORA_SPEED_CROSSDIGI_PRESET, server.arg(PREF_LORA_SPEED_CROSSDIGI_PRESET).toInt());
  }
  if (server.hasArg(PREF_LORA_TX_POWER_CROSSDIGI_PRESET)) {
    preferences.putInt(PREF_LORA_TX_POWER_CROSSDIGI_PRESET, server.arg(PREF_LORA_TX_POWER_CROSSDIGI_PRESET).toInt());
  }
  if (server.hasArg(PREF_LORA_RX_ON_FREQUENCIES_PRESET)) {
    preferences.putInt(PREF_LORA_RX_ON_FREQUENCIES_PRESET, server.arg(PREF_LORA_RX_ON_FREQUENCIES_PRESET).toInt());
  }
  // APRS station settings
  if (server.hasArg(PREF_APRS_CALLSIGN) && !server.arg(PREF_APRS_CALLSIGN).isEmpty()){
    preferences.putString(PREF_APRS_CALLSIGN, server.arg(PREF_APRS_CALLSIGN));
  }
  if (server.hasArg(PREF_APRS_SYMBOL_TABLE) && !server.arg(PREF_APRS_SYMBOL_TABLE).isEmpty()){
    preferences.putString(PREF_APRS_SYMBOL_TABLE, server.arg(PREF_APRS_SYMBOL_TABLE));
  }
  if (server.hasArg(PREF_APRS_SYMBOL) && !server.arg(PREF_APRS_SYMBOL).isEmpty()){
    preferences.putString(PREF_APRS_SYMBOL, server.arg(PREF_APRS_SYMBOL));
  }
  if (server.hasArg(PREF_APRS_RELAY_PATH)){
    preferences.putString(PREF_APRS_RELAY_PATH, server.arg(PREF_APRS_RELAY_PATH));
  }
  if (server.hasArg(PREF_APRS_COMMENT)){
    preferences.putString(PREF_APRS_COMMENT, server.arg(PREF_APRS_COMMENT));
  }
  if (server.hasArg(PREF_APRS_LATITUDE_PRESET)){
    preferences.putString(PREF_APRS_LATITUDE_PRESET, server.arg(PREF_APRS_LATITUDE_PRESET));
  }
  if (server.hasArg(PREF_APRS_LONGITUDE_PRESET)){
    preferences.putString(PREF_APRS_LONGITUDE_PRESET, server.arg(PREF_APRS_LONGITUDE_PRESET));
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_INTERVAL)){
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_INTERVAL, server.arg(PREF_TNC_SELF_TELEMETRY_INTERVAL).toInt());
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_MIC)){
    preferences.putInt(PREF_TNC_SELF_TELEMETRY_MIC, server.arg(PREF_TNC_SELF_TELEMETRY_MIC).toInt());
  }
  if (server.hasArg(PREF_TNC_SELF_TELEMETRY_PATH)){
    preferences.putString(PREF_TNC_SELF_TELEMETRY_PATH, server.arg(PREF_TNC_SELF_TELEMETRY_PATH));
  }

  // Smart Beaconing settings 
  if (server.hasArg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET)){
    preferences.putInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET, server.arg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MIN_INTERVAL_PRESET)){
    preferences.putInt(PREF_APRS_SB_MIN_INTERVAL_PRESET, server.arg(PREF_APRS_SB_MIN_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MAX_INTERVAL_PRESET)){
    //preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, server.arg(PREF_APRS_SB_MAX_INTERVAL_PRESET).toInt());
    int i = server.arg(PREF_APRS_SB_MAX_INTERVAL_PRESET).toInt();
    if (i <= preferences.getInt(PREF_APRS_SB_MIN_INTERVAL_PRESET)) i = preferences.getInt(PREF_APRS_SB_MIN_INTERVAL_PRESET) +1;
    preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, i);
  }
  if (server.hasArg(PREF_APRS_SB_MIN_SPEED_PRESET)){
    preferences.putInt(PREF_APRS_SB_MIN_SPEED_PRESET, server.arg(PREF_APRS_SB_MIN_SPEED_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_MAX_SPEED_PRESET)){
    //preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, server.arg(PREF_APRS_SB_MAX_SPEED_PRESET).toInt());
    int i = server.arg(PREF_APRS_SB_MAX_SPEED_PRESET).toInt();
    if (i <= preferences.getInt(PREF_APRS_SB_MIN_SPEED_PRESET)) i = preferences.getInt(PREF_APRS_SB_MIN_SPEED_PRESET) +1;
    preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, i);
  }
  if (server.hasArg(PREF_APRS_SB_ANGLE_PRESET)){
    preferences.putDouble(PREF_APRS_SB_ANGLE_PRESET, server.arg(PREF_APRS_SB_ANGLE_PRESET).toDouble());
  }
  if (server.hasArg(PREF_APRS_SB_TURN_SLOPE_PRESET)){
    preferences.putInt(PREF_APRS_SB_TURN_SLOPE_PRESET, server.arg(PREF_APRS_SB_TURN_SLOPE_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_SB_TURN_TIME_PRESET)){
    preferences.putInt(PREF_APRS_SB_TURN_TIME_PRESET, server.arg(PREF_APRS_SB_TURN_TIME_PRESET).toInt());
  }
 
  preferences.putBool(PREF_APRSIS_EN, server.hasArg(PREF_APRSIS_EN));
  if (server.hasArg(PREF_APRSIS_SERVER_NAME)){
    preferences.putString(PREF_APRSIS_SERVER_NAME, server.arg(PREF_APRSIS_SERVER_NAME));
  }
  if (server.hasArg(PREF_APRSIS_SERVER_PORT)){
    preferences.putInt(PREF_APRSIS_SERVER_PORT, server.arg(PREF_APRSIS_SERVER_PORT).toInt());
  }
  if (server.hasArg(PREF_APRSIS_FILTER)){
    preferences.putString(PREF_APRSIS_FILTER, server.arg(PREF_APRSIS_FILTER));
  }
  if (server.hasArg(PREF_APRSIS_CALLSIGN)){
    preferences.putString(PREF_APRSIS_CALLSIGN, server.arg(PREF_APRSIS_CALLSIGN));
  }
  if (server.hasArg(PREF_APRSIS_PASSWORD)){
    preferences.putString(PREF_APRSIS_PASSWORD, server.arg(PREF_APRSIS_PASSWORD));
  }
  preferences.putBool(PREF_APRSIS_ALLOW_INET_TO_RF, server.hasArg(PREF_APRSIS_ALLOW_INET_TO_RF));
  

  preferences.putBool(PREF_APRS_SHOW_BATTERY, server.hasArg(PREF_APRS_SHOW_BATTERY));
  preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY, server.hasArg(PREF_ENABLE_TNC_SELF_TELEMETRY));
  preferences.putBool(PREF_APRS_SHOW_ALTITUDE, server.hasArg(PREF_APRS_SHOW_ALTITUDE));
  preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET, server.hasArg(PREF_APRS_FIXED_BEACON_PRESET));
  preferences.putBool(PREF_APRS_GPS_EN, server.hasArg(PREF_APRS_GPS_EN));
  preferences.putBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS, server.hasArg(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS));
  preferences.putBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS, server.hasArg(PREF_GPS_ALLOW_SLEEP_WHILE_KISS));
  preferences.putBool(PREF_APRS_SHOW_CMT, server.hasArg(PREF_APRS_SHOW_CMT));
  preferences.putBool(PREF_APRS_COMMENT_RATELIMIT_PRESET, server.hasArg(PREF_APRS_COMMENT_RATELIMIT_PRESET));

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  
}

void handle_saveDeviceCfg(){
  preferences.putBool(PREF_DEV_BT_EN, server.hasArg(PREF_DEV_BT_EN));
  preferences.putBool(PREF_DEV_OL_EN, server.hasArg(PREF_DEV_OL_EN));
  if (server.hasArg(PREF_DEV_SHOW_RX_TIME)){
    preferences.putInt(PREF_DEV_SHOW_RX_TIME, server.arg(PREF_DEV_SHOW_RX_TIME).toInt());
  }
  // Manage OLED Timeout
  if (server.hasArg(PREF_DEV_SHOW_OLED_TIME)){
    preferences.putInt(PREF_DEV_SHOW_OLED_TIME, server.arg(PREF_DEV_SHOW_OLED_TIME).toInt());
  }
  preferences.putBool(PREF_DEV_AUTO_SHUT, server.hasArg(PREF_DEV_AUTO_SHUT));
  if (server.hasArg(PREF_DEV_AUTO_SHUT_PRESET)){
    preferences.putInt(PREF_DEV_AUTO_SHUT_PRESET, server.arg(PREF_DEV_AUTO_SHUT_PRESET).toInt());
  } 
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
}

[[noreturn]] void taskWebServer(void *parameter) {
  auto *webServerCfg = (tWebServerCfg*)parameter;
  apSSID = webServerCfg->callsign + " AP";

  server.on("/", handle_Index);
  server.on("/favicon.ico", handle_NotFound);
  server.on("/style.css", handle_Style);
  server.on("/js.js", handle_Js);
  server.on("/scan_wifi", handle_ScanWifi);
  server.on("/save_wifi_cfg", handle_SaveWifiCfg);
  server.on("/reboot", handle_Reboot);
  server.on("/beacon", handle_Beacon);
  server.on("/shutdown", handle_Shutdown);
  server.on("/cfg", handle_Cfg);
  server.on("/received_list", handle_ReceivedList);
  server.on("/save_aprs_cfg", handle_SaveAPRSCfg);
  server.on("/save_device_cfg", handle_saveDeviceCfg);
  server.on("/restore", handle_Restore);
  server.on("/update", HTTP_POST, []() {
    syslog_log(LOG_WARNING, String("Update finished. Status: ") + (!Update.hasError() ? "Ok" : "Error"));
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    delay(500);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      rf95.sleep(); // disable rf95 before update
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        syslog_log(LOG_ERR, String("Update begin error: ") + Update.errorString());
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        syslog_log(LOG_ERR, String("Update error: ") + Update.errorString());
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        syslog_log(LOG_WARNING, String("Update Success: ") + String((int)upload.totalSize));
      } else {
        syslog_log(LOG_ERR, String("Update error: ") + Update.errorString());
        Update.printError(Serial);
      }
    }
  });
  server.onNotFound(handle_NotFound);

  String wifi_password = preferences.getString(PREF_WIFI_PASSWORD);
  String wifi_ssid = preferences.getString(PREF_WIFI_SSID);
  if (preferences.getString(PREF_AP_PASSWORD).length() > 8) {
    // 8 characters is requirements for WPA2
    apPassword = preferences.getString(PREF_AP_PASSWORD);
  } else {
    apPassword = defApPassword;
  }
  if (!wifi_ssid.length()){
    WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  } else {
    int retryWifi = 0;
    WiFi.begin(wifi_ssid.c_str(), wifi_password.length() ? wifi_password.c_str() : nullptr);
    Serial.println("Connecting to " + wifi_ssid);
    // Set power to minimum (max 20)
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
    esp_wifi_set_max_tx_power(8);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print("Not connected: ");
      Serial.println((int)WiFi.status());
      Serial.print("Retry: ");
      Serial.println(retryWifi);
      vTaskDelay(500/portTICK_PERIOD_MS);
      retryWifi += 1;
      if (retryWifi > 60) {
        WiFi.softAP(apSSID.c_str(), apPassword.c_str());
        //WiFi.softAP(apSSID.c_str(), "password");
        Serial.println("Unable to connect to to wifi. Starting AP");
        Serial.print("SSID: ");
        Serial.print(apSSID.c_str());
        Serial.print(" Password: ");
        Serial.println(apPassword.c_str());
        // Set power to minimum (max 20)
        esp_wifi_set_max_tx_power(8);
        break;
      }
    }

    //Serial.print("WiFi Mode: ");
    //Serial.println(WiFi.getMode());
    if (WiFi.getMode() == 3){
      Serial.println("Running AP. IP: " + WiFi.softAPIP().toString());
      apEnabled=true;
      infoApName = apSSID.c_str();
      infoApPass = apSSID.c_str();
      infoApAddr = WiFi.softAPIP().toString();
    } else if (WiFi.getMode() == 1) {
      // Save some battery
      //WiFi.setSleep(true);
      esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
      Serial.println("Connected. IP: " + WiFi.localIP().toString());
      apConnected=true;
      infoApName = wifi_ssid.c_str();
      infoApPass = wifi_password.c_str();
      infoApAddr = WiFi.localIP().toString();
    } else {
      Serial.println("WiFi Mode: " + WiFi.getMode());
    }

    #ifdef ENABLE_SYSLOG
      syslog.server(SYSLOG_IP, 514);
      syslog.deviceHostname(webServerCfg->callsign.c_str());
      syslog.appName("TTGO");
      syslog.defaultPriority(LOG_KERN);
      syslog_log(LOG_INFO, "Connected. IP: " + WiFi.localIP().toString());
    #endif
    configTime(0, 0, "pool.ntp.org");
    #ifdef ENABLE_SYSLOG
      struct tm timeinfo{};
      if(!getLocalTime(&timeinfo)){
        syslog_log(LOG_WARNING, "Failed to obtain time");
      } else {
        char buf[64];
        strftime(buf, 64, "%A, %B %d %Y %H:%M:%S", &timeinfo);
        syslog_log(LOG_INFO, String("Time: ") + String(buf));
      }
    #endif
  }

  server.begin();
  #ifdef KISS_PROTOCOL
    tncServer.begin();
  #endif
  gpsServer.begin();
  if (MDNS.begin(webServerCfg->callsign.c_str())) {
    MDNS.setInstanceName(webServerCfg->callsign + " TTGO LoRa APRS TNC " + TXFREQ + "MHz");
    MDNS.addService("http", "tcp", 80);
    #ifdef KISS_PROTOCOL
      MDNS.addService("kiss-tnc", "tcp", NETWORK_TNC_PORT);
    #endif
  }

  webListReceivedQueue = xQueueCreate(4,sizeof(tReceivedPacketData *));


  tReceivedPacketData *receivedPacketData = nullptr;

  WiFiClient aprs_is_client;
  uint32_t t_connect_apprsis_again = 0L;
  String aprs_callsign = webServerCfg->callsign;
  aprsis_host.trim();
  aprsis_filter.trim();
  aprsis_password.trim();
  if (aprsis_callsign.length() < 3 || aprsis_callsign.length() > 9)
    aprsis_callsign = "";
  if (aprsis_callsign.isEmpty()) aprsis_callsign = aprs_callsign;
  if (aprsis_callsign.length() < 3 || aprsis_callsign.length() > 9)
    aprsis_callsign = "";
  aprsis_callsign.toUpperCase(); aprsis_callsign.trim();

  // sanity check
  if (aprsis_enabled) {
    if (aprsis_callsign.isEmpty() || aprsis_host.isEmpty() || aprsis_port == 0) {
      aprsis_enabled = false;
   } else {
      const char *p = aprsis_callsign.c_str();
      for (; *p && aprsis_enabled; p++) {
        if (*p == '-') {
          int len = strlen(p+1);
          if (len < 1 || len > 2)
            aprsis_enabled = false;
        } else if ( !((*p >= 'A' && *p <= 'Z') || (*p >= '0' && *p <= '9')) )
            aprsis_enabled = false;
      }
    }
  }

  while (true){
    server.handleClient();
    if (xQueueReceive(webListReceivedQueue, &receivedPacketData, (1 / portTICK_PERIOD_MS)) == pdPASS) {
      auto *receivedPacketToQueue = new tReceivedPacketData();
      receivedPacketToQueue->packet = new String();
      receivedPacketToQueue->packet->concat(*receivedPacketData->packet);
      receivedPacketToQueue->RSSI = receivedPacketData->RSSI;
      receivedPacketToQueue->SNR = receivedPacketData->SNR;
      receivedPacketToQueue->rxTime = receivedPacketData->rxTime;
      receivedPackets.push_back(receivedPacketToQueue);
      if (receivedPackets.size() > MAX_RECEIVED_LIST_SIZE){
        auto *packetDataToDelete = receivedPackets.front();
        delete packetDataToDelete->packet;
        delete packetDataToDelete;
        receivedPackets.pop_front();
      }
      delete receivedPacketData->packet;
      delete receivedPacketData;
    }


    if (aprsis_enabled) {
      boolean err = true;
      if (WiFi.getMode() == 1) {
        if (WiFi.status() != WL_CONNECTED) { aprsis_status = "Error: no internet"; goto on_err; } else { if (aprsis_status == "Error: no internet") aprsis_status = "Internet available"; }
        if (!aprs_is_client.connected() && t_connect_apprsis_again < millis()) {
          aprsis_status = "Connecting";
          aprs_is_client.connect(aprsis_host.c_str(), aprsis_port);
          if (!aprs_is_client.connected()) { aprsis_status = "Error: connect failed"; goto on_err; }
          aprsis_status = "Connected. Waiting for greeting.";
          uint32_t t_start = millis();
          while (!aprs_is_client.available() && (millis()-t_start) < 25000) delay(100);
          if (aprs_is_client.available()) {
	    // check 
	    String s = aprs_is_client.readStringUntil('\n');
	    if (s.isEmpty() || !s.startsWith("#")) { aprsis_status = "Error: unexpected greeting"; goto on_err; }
	  } else { aprsis_status = "Error: No response"; goto on_err; }
          aprsis_status = "Login";
	  char buffer[1024];
	  sprintf(buffer, "user %s pass %s TTGO-T-Beam-LoRa-APRS 0.1%s%s\r\n", aprsis_callsign.c_str(), aprsis_password.c_str(), aprsis_filter.isEmpty() ? " filter " : "", aprsis_filter.isEmpty() ? aprsis_filter.c_str() : "");
          aprs_is_client.print(String(buffer));
          t_start = millis();
          while (!aprs_is_client.available() && (millis()-t_start) < 25000) delay(100);
          aprsis_status = "Logged in";
          if (aprs_is_client.available()) {
	    // check 
	    String s = aprs_is_client.readStringUntil('\n');
	    if (s.isEmpty() || !s.startsWith("#")) { aprsis_status = "Error: unexpected reponse on login"; goto on_err; }
	    if (s.indexOf(" verified") == -1) { aprsis_status = "Error: Login denied: " + s; aprsis_status.trim(); goto on_err; }
	  } else { aprsis_status = "Error: No response"; goto on_err; }
        }
        if (!aprs_is_client.connected())
          goto on_err;
        //aprsis_status = "OK";
        if (aprs_is_client.available()) {
          //aprsis_status = "OK, reading";
          String s = aprs_is_client.readStringUntil('\n');
          if (!s) goto on_err;
          //aprsis_status = "OK";
          s.trim();
          if (s.isEmpty()) goto on_err;
	  char *header_end = strchr(s.c_str(), ':');
          if (*(s.c_str()) != '#' && header_end) {
	    boolean do_not_send = false;
	    char *q;
	    // do not interprete packets coming back from aprs-is net (either our source call, or if we have repeated it with one of our calls
            if (s.startsWith(aprs_callsign + '>') || s.startsWith(aprsis_callsign + '>')) {
	      // sender is our call
	      do_not_send = true;
	    } else {
	      // packet has our call in i.E. ...,qAR,OURCALL:...
	      q = strstr(s.c_str(), (',' + aprsis_callsign + ':').c_str());
	      if (q && q < header_end)
	        do_not_send = true;
	    }
	    if (!do_not_send) {
	      for (int i = 0; i < 2; i++) {
	        String call = (i == 0 ? aprs_callsign : aprsis_callsign);
		// digipeated frames look like "..,DL9SAU,DL1AAA,DL1BBB*,...", or "..,DL9SAU*,DL1AAA,DL1BBB,.."
	        if (((q = strstr(s.c_str(), (',' + call + '*').c_str())) || (q = strstr(s.c_str(), (',' + call + ',').c_str()))) && q < header_end) {
                  char *digipeatedflag = strchr(q, '*');
	          if (digipeatedflag && digipeatedflag < header_end && digipeatedflag > q)
	            do_not_send = true;
	        }
	      }
	    }
	    if (!do_not_send) {
	      // generate third party packet. Use aprs_callsign (deriving from webServerCfg->callsign), because aprsis_callsign may have a non-aprs (but only aprsis-compatible) ssid like '-L4'
              String third_party_packet = generate_third_party_packet(aprs_callsign, s);
              if (!third_party_packet.isEmpty()) {
#ifdef KISS_PROTOCOL
                sendToTNC(third_party_packet);
#endif
                if (lora_tx_enabled && aprsis_data_allow_inet_to_rf) {
		  // not query or aprs-message addressed to our call (check both, aprs_callsign and aprsis_callsign)=
	          // Format: "..::DL9SAU-15:..."
		  // check is here, because we may like to see those packets via kiss (sent above)
		  if (!do_not_send) {
		    q = header_end + 1;
		    if (*q == ':' && strlen(q) > 10 && q[10] == ':' &&
                         ((!strncmp(q+1, aprs_callsign.c_str(), aprs_callsign.length()) && (aprs_callsign.length() == 9 || q[9] == ' ')) ||
                         (!strncmp(q+1, aprsis_callsign.c_str(), aprsis_callsign.length()) && (aprsis_callsign.length() == 9 || q[9] == ' ')) ))
		        do_not_send = true;
		  }
                  if (!do_not_send)
                    loraSend(txPower, lora_freq, lora_speed, third_party_packet);
	        }
              }
            }
          }
        }
        if (to_aprsis_data) {
	  // copy(). We are threaded..
	  String data = String(to_aprsis_data);
	  // clear queue
          to_aprsis_data = "";
          data.trim();
          char *p = strchr(data.c_str(), '>');
          char *q;
          // some plausibility checks.
          if (p && p > data.c_str() && (q = strchr(p+1, ':'))) {
            // Due to http://www.aprs-is.net/IGateDetails.aspx , never gate third-party traffic contining TCPIP or TCPXX
            // IGATECALL>APRS,GATEPATH:}FROMCALL>TOCALL,TCPIP,IGATECALL*:original packet data
            char *r; char *s;
            if (!(q[1] == '}' && (r = strchr(q+2, '>')) && ((s = strstr(r+1, ",TCPIP,")) || (s = strstr(r+1, ",TCPXX,"))) && strstr(s+6, "*:"))) {
	      char buf[256];
	      int len = (q-data.c_str());
	      if (len > 0 && len < sizeof(buf)) {
	        strncpy(buf, data.c_str(), len);
                buf[len] = 0;
		String s_data = String(buf) + (lora_tx_enabled ? ",qAR," : ",qAO,") + aprsis_callsign + q + "\r\n";
                aprsis_status = "OK, sending: " + s_data; aprsis_status.trim();
                aprs_is_client.print(s_data);
	      }
            }
          }
          //aprsis_status = "OK";
        }
        err = false;

        if (err) {
on_err:
	  aprs_is_client.stop();
	  if (!aprsis_status.startsWith("Error: "))
            aprsis_status = "Disconnected";
	  if (t_connect_apprsis_again <=  millis())
	    t_connect_apprsis_again = millis() + 60000;
          to_aprsis_data = "";
        }
      }
    }

    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}


String generate_third_party_packet(String callsign, String packet_in)
{
  String packet_out = "";
  const char *s = packet_in.c_str();
  char *p = strchr(s, '>');
  char *q = strchr(s, ',');
  char *r = strchr(s, ':');
  char fromtodest[20]; // room for max (due to spec) 'DL9SAU-15>APRSXX-NN' + \0
  if (p > s && p < q && q < r && (q-s) < sizeof(fromtodest)) {
    r++;
    strncpy(fromtodest, s, q-s);
    fromtodest[(q-s)] = 0;
    packet_out = callsign + ">APRS:}" + fromtodest + ",TCPIP," + callsign + "*:" + r;
                             // ^ 3rd party traffic should be addressed directly (-> not to WIDE2-1 or so)
  }
  return packet_out;
}
