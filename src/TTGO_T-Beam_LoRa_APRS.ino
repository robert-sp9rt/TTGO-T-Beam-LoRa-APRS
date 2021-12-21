// Tracker for LoRA APRS
// from OE1ACM and OE3CJB redesigned by SQ9MDD
// KISS ans Bluetooth by SQ5RWU
// TTGO T-Beam v1.0 only
//
// licensed under CC BY-NC-SA
// Includes
//#include <TTGO_T-Beam_LoRa_APRS_config.h> // to config user parameters
#include <Arduino.h>
#include <SPI.h>
#include <BG_RF95.h>         // library from OE1ACM
#include <math.h>
#include <driver/adc.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <axp20x.h>
#include "taskGPS.h"
#include "version.h"
#include "preference_storage.h"
#include "syslog_log.h"

#ifdef KISS_PROTOCOL
  #include "taskTNC.h"
#endif
#ifdef ENABLE_WIFI
  #include "taskWebServer.h"
#endif

// oled address
#define SSD1306_ADDRESS 0x3C

// SPI config
#define SPI_sck 5
#define SPI_miso 19
#define SPI_mosi 27
#define SPI_ss 18

// IO config
#ifdef T_BEAM_V1_0
  #define I2C_SDA 21
  #define I2C_SCL 22    
  #define BUTTON  38                //pin number for Button on TTGO T-Beam
  #define BUZZER 15                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif T_BEAM_V0_7
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON  39                //pin number for Button on TTGO T-Beam
  #define BUZZER 15                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
/* Original LORA32 V2.1 Setup
#elif LORA32_21
  #define I2C_SDA 4
  #define I2C_SCL 15
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
*/
#elif LORA32_21                     // Modified as in #47
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif LORA32_2
  #define I2C_SDA 21
  #define I2C_SCL 22
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif LORA32_1
  #define I2C_SDA 21
  #define I2C_SCL 22 
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif HELTEC_V1
  #define I2C_SDA 4
  #define I2C_SCL 15 
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#elif HELTEC_V2
  #define I2C_SDA 4
  #define I2C_SCL 15    
  #define BUTTON 2                  //pin number for BUTTO
  #define BUZZER 13                 // enter your buzzer pin gpio
  const byte TXLED  = 4;            //pin number for LED on TX Tracker
#endif

// Variables for LoRa settings
#ifdef LORA_SPEED_1200
  ulong lora_speed = 1200;
#else
  ulong lora_speed = 300;
#endif

#ifdef	TXFREQ
  double lora_freq = TXFREQ;
#else
  double lora_freq = 433.775;
#endif

ulong lora_speed_cross_digi = 1200;
double lora_freq_cross_digi = 433.900;

// Variables for APRS packaging
String Tcall;                       //your Call Sign for normal position reports
String aprsSymbolTable = APRS_SYMBOL_TABLE;
String aprsSymbol = APRS_SYMBOL;
String relay_path;
String aprsComment = MY_COMMENT;
String aprsLatPreset = LATITUDE_PRESET;
String aprsLonPreset = LONGITUDE_PRESET;
#if defined(T_BEAM_V1_0) || defined(T_BEAM_V0_7)
boolean gps_state = true;
#else
boolean gps_state = false;
#endif
boolean key_up = true;
boolean t_lock = false;
boolean fixed_beacon_enabled = false;
boolean show_cmt = true;
// Telemetry sequence, current value
int tel_sequence;
// Telemetry path
String tel_path;

#ifdef SHOW_ALT
  boolean showAltitude = true;
#else
  boolean showAltitude = false;
#endif
#ifdef SHOW_BATT
  boolean showBattery = true;
#else
  boolean showBattery = false;
#endif
#ifdef ENABLE_TNC_SELF_TELEMETRY
  boolean enable_tel = true;
#else
  boolean enable_tel = false;
#endif
  // Telemetry interval, seconds
#ifdef TNC_SELF_TELEMETRY_INTERVAL
  int tel_interval = TNC_SELF_TELEMETRY_INTERVAL;
#else
  int tel_interval = 3600;
#endif
#ifdef TNC_SELF_TELEMETRY_MIC
  int tel_mic = 1; // telemetry as "T#MIC"
#else
  int tel_mic = 0; // telemetry as "T#001"
#endif
#ifdef ENABLE_BLUETOOTH
  boolean enable_bluetooth = true;
#else
  boolean enable_bluetooth = false;
#endif
#ifdef ENABLE_OLED
  boolean enabled_oled = true;
#else
  boolean enabled_oled = false;
#endif

// Variables and Constants
String loraReceivedFrameString = "";      //data on buff is copied to this string
String Outputstring = "";
String outString="";                      //The new Output String with GPS Conversion RAW
String LongShown="";
String LatShown="";
String LongFixed="";
String LatFixed="";

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
  time_t nextTelemetryFrame;
#endif

//byte arrays
byte  lora_TXBUFF[BG_RF95_MAX_MESSAGE_LEN];      //buffer for packet to send
byte  lora_RXBUFF[BG_RF95_MAX_MESSAGE_LEN];      //buffer for packet to send

//byte Variables
byte  lora_TXStart;          //start of packet data in TXbuff
byte  lora_TXEnd;            //end of packet data in TXbuff
byte  lora_FTXOK;            //flag, set to 1 if TX OK
byte  lora_TXPacketType;     //type number of packet to send
byte  lora_TXDestination;    //destination address of packet to send
byte  lora_TXSource;         //source address of packet received
byte  lora_FDeviceError;     //flag, set to 1 if RFM98 device error
byte  lora_TXPacketL;        //length of packet to send, includes source, destination and packet type.

unsigned long lastTX = 0L;
float BattVolts;
float InpVolts;

// variables for smart beaconing
#ifdef SB_ALGO_KENWOOD
  // Kenwood scales better on lower speed.
  ulong sb_min_interval = 120000L;
  ulong sb_max_interval = 1800000L;
  float sb_min_speed = 5;
  float sb_max_speed = 70;
  float sb_angle = 28;
#else
  ulong sb_min_interval = 60000L;
  ulong sb_max_interval = 360000L;
  float sb_min_speed = 0;
  float sb_max_speed = 30;
  float sb_angle = 30;                      // angle to send packet at smart beaconing
#endif
int sb_turn_slope = 26;			// kenwood example: 26 in mph. Yaesu suggests 26 in high speed car, 11 car in low/mid speed, 7 on walking;
					// TS 7 if <= 20km/h, TS 11 if <= 50km/h, TS 26 else.
int sb_turn_time = 30;			// min. 30s between transmissions (kenwood example)

float average_speed[5] = {0,0,0,0,0}, average_speed_final=0;
float old_course = 0, new_course = 0;
int point_avg_speed = 0, point_avg_course = 0;

ulong nextTX=60000L;                  // preset time period between TX = 60000ms = 60secs = 1min

ulong time_to_refresh = 0;
ulong next_fixed_beacon = 0;
ulong fix_beacon_interval = FIX_BEACON_INTERVAL;
ulong showRXTime = SHOW_RX_TIME;
ulong time_delay = 0;
ulong shutdown_delay = 0;
ulong shutdown_delay_time = 10000;
ulong shutdown_countdown_timer = 0;
boolean shutdown_active =true;
boolean shutdown_countdown_timer_enable = false;
boolean shutdown_usb_status_bef = false;

// Variables required to Power Save OLED
// With "Display dimmer enabled" it will turn OLED off after some time
// if the checkbox is disabled the display stays OFF
uint oled_timeout = SHOW_OLED_TIME; // OLED Timeout
bool tempOled = true; // Turn ON OLED at first startup
ulong oled_timer;

// Variable to manually send beacon from html page
bool manBeacon = false;

// Variable to show AP settings on OLED
bool apEnabled = false;
bool apConnected = false;
String infoApName = "";
String infoApPass = "";
String infoApAddr = "";

#define ANGLE_AVGS 3                  // angle averaging - x times
float average_course[ANGLE_AVGS];
float avg_c_y, avg_c_x;

#ifdef TXDISABLE		// define TXDISABLE if you like to ensure that we never TX (i.e. if we are behind an rx-amplifier)
boolean lora_tx_enabled = false;
uint8_t txPower = 0;
#else
boolean lora_tx_enabled = true;
#ifdef TXdbmW
uint8_t txPower = TXdbmW;
#else
uint8_t txPower = 23;
#endif
#endif


// may be configured
boolean rate_limit_message_text = true;		// ratelimit adding messate text (-> saves airtime)
boolean lora_automatic_cr_adaption = false;	// automatic CR adaption (should be configurable in Web-Interface).
						// We'll count users (except own digipeated packets), and if we're alone, CR4/8 doesn't disturb anyone.
						// If we have a high load on the channel, we'll decrease up to CR4/5.
						// You may set this to off if you are a fixed station / repeater / gateway
						// This may become set to true by default, after it proves it behaves good to our network
uint8_t lora_digipeating_mode = 1;		// Digipeating: 0: disabled (recommended if the device should not do repeating decisions, and even more, if you have attached a normal aprs digipeating software via kiss). 1: if own call addressed (recommended for users) 2: act as WIDE1 fill-in digi (recommended for standalone fill-in-digi). 3: act as a simple stupid WIDE2 digi
uint8_t lora_cross_digipeating_mode = 0;	// 0: disable cross freq digipeating. 1: send on both frequencies. 2: send only on cross frequency
boolean lora_add_snr_rssi_to_path = true;	// Add snr+rssi to path. May become default, after it proves it behaves good to our network

#ifdef KISS_PROTOCOL
bool acceptOwnPositionReportsViaKiss = true;		// may be a web-configurable variable true: Switches off local beacons as long as a kiss device is sending positions with our local callsign. false: filters out position packets with own callsign coming from kiss (-> do not send to LoRa).
boolean gps_allow_sleep_while_kiss = true;		// may be a web-configurable variable. If user has a kiss device attached via kiss which sends positions with own call, we don't need our gps to be turned on -> We pause sending positions by ourself (neither fixed nor smart beaconing). Except: user has a display attached to this tracker, he'll will be able to see his position because our gps does not go to sleep (-> set this to false). Why sleep? Energy saving

// do not configure
uint32_t time_last_own_position_via_kiss_received = 0L;	// kiss client sends position report with our call+ssid. Remember when.
uint32_t time_last_frame_via_kiss_received = 0L;	// kiss client sends aprs-text-messages with our call+ssid. Remember when.
boolean kiss_client_came_via_bluetooth = false;
#endif

// do not configure
boolean dont_send_own_position_packets = false;		// dynamicaly set if kiss device sends position. Maybe there are other usecases (-> kiss-independent)
boolean gps_state_before_autochange = false;		// remember gps state before autochange
uint32_t time_last_lora_frame_received = 0L;
uint32_t time_last_own_text_message_via_kiss_received = 0L;
uint32_t time_lora_automaic_cr_adoption_rx_measurement_window = 0L;
uint16_t lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot = 0;
char lora_TXBUFF_for_digipeating[BG_RF95_MAX_MESSAGE_LEN+1] = "";		// buffer for digipeating
time_t time_lora_TXBUFF_for_digipeating_was_filled = 0L;

#ifdef ENABLE_WIFI
  tWebServerCfg webServerCfg;
#endif

static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;
#ifdef T_BEAM_V1_0
  AXP20X_Class axp;
#endif

// checkRX
uint8_t loraReceivedLength = sizeof(lora_RXBUFF);

// Singleton instance of the radio driver
BG_RF95 rf95(18, 26);        // TTGO T-Beam has NSS @ Pin 18 and Interrupt IO @ Pin26

// initialize OLED display
#define OLED_RESET 16         // not used
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// + FUNCTIONS-----------------------------------------------------------+//

char *ax25_base91enc(char *s, uint8_t n, uint32_t v){
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */
  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }
  return(s);
}

void prepareAPRSFrame(){
  String helper;
  String Altx;
  String Speedx, Coursex;
  char helper_base91[] = {"0000\0"};
  double Tlat=52.0000, Tlon=20.0000;
  double Tspeed=0, Tcourse=0;
  uint32_t aprs_lat, aprs_lon;
  int i;
  long Talt;
  Tlat=gps.location.lat();
  Tlon=gps.location.lng();
  Tcourse=gps.course.deg();
  Tspeed=gps.speed.knots();
  aprs_lat = 900000000 - Tlat * 10000000;
  aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
  aprs_lon = 900000000 + Tlon * 10000000 / 2;
  aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  outString = "";
  outString += Tcall;

  outString += ">APLOX1";
  if (!relay_path.isEmpty()){
    if (relay_path.length() < 3) {
      int ssid = relay_path.toInt();
      if (ssid > 0 && ssid <= /* 15 // no, max hop 3 */ 3) {
        char buf[4];
        sprintf(buf, "-%d", ssid);
        outString += buf;
      }
    } else {
      outString = outString + "," + relay_path;
    }
  }
  outString += ":";

  if (
#ifdef ENABLE_BLUETOOTH
       SerialBT.hasClient() ||
#endif
       ((time_last_own_text_message_via_kiss_received + 24*60*60*1000L) > millis())
     )
    outString += "=";
  else
    outString += "!";

  if(gps_state && gps.location.isValid()){
    outString += aprsSymbolTable;
    ax25_base91enc(helper_base91, 4, aprs_lat);
    for (i = 0; i < 4; i++) {
      outString += helper_base91[i];
    }
    ax25_base91enc(helper_base91, 4, aprs_lon);
    for (i = 0; i < 4; i++) {
      outString += helper_base91[i];
    }
    outString += aprsSymbol;
    ax25_base91enc(helper_base91, 1, (uint32_t) Tcourse / 4);
    outString += helper_base91[0];
    ax25_base91enc(helper_base91, 1, (uint32_t) (log1p(Tspeed) / 0.07696));
    outString += helper_base91[0];
    outString += "H";

    if (showAltitude){
      Talt = gps.altitude.feet();
      char buf[7];
      outString += "/A=";
      if (Talt > 999999) Talt=999999;
      else if (Talt < -99999) Talt=-99999;
      sprintf(buf, "%06ld", Talt);
      outString += buf;
    }
  }else{  //fixed position not compresed
    outString += aprsLatPreset;
    outString += aprsSymbolTable;
    outString += aprsLonPreset;
    outString += aprsSymbol;
  }

  if(show_cmt){
    static uint8_t comments_added = 0;
    static uint32_t time_comment_added = 0L;
    if (!rate_limit_message_text || (time_comment_added + (gps_state ? sb_max_interval : fix_beacon_interval) < millis()))
      comments_added = 0;
    if ((comments_added++ % 10) == 0) {
      outString += aprsComment;
      time_comment_added = millis();
    }
  }

  if(showBattery){
    outString += " Batt=";
    outString += String((BattVolts > 1.0 ? BattVolts : InpVolts), 2);
    outString += ("V");
  }

  #ifdef KISS_PROTOCOL
    sendToTNC(outString);
  #endif
}

#ifdef BUZZER
/**
 * Buzzer sound playback
 * @param melody - must be an array. Consisting of an even number of values. frequency and duration
 * @param array_size - number of elements in the array
 */
void buzzer(int* melody, int array_size){
  for(int i=0; i<array_size; i+=2){
      ledcWriteTone(0, *melody);
      melody++;
      delay(*melody);
      melody++;
  }
  ledcWriteTone(0,0); // turn off buzzer
}
#endif

void sendpacket(){
  #ifdef BUZZER
    int melody[] = {1000, 50, 800, 100};
    buzzer(melody, sizeof(melody)/sizeof(int));
  #endif
  batt_read();
  prepareAPRSFrame();
  loraSend(txPower, lora_freq, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
}

/**
 * Send message as APRS LoRa packet
 * @param lora_LTXPower
 * @param lora_FREQ
 * @param message
 */
// Feauture request: add param lora_speed. Currently, we need a variable for storing the old speed, and affer loraSend(), we have to revert :(
void loraSend(byte lora_LTXPower, float lora_FREQ, const String &message) {
  if (!lora_tx_enabled)
    return;
  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, LOW);
  #endif
  lastTX = millis();

  int messageSize = min(message.length(), sizeof(lora_TXBUFF) - 1);
  message.toCharArray((char*)lora_TXBUFF, messageSize + 1, 0);
  if(lora_speed==1200){
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf512);
  }
  else if(lora_speed==610){
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf1024);
  }
  else if(lora_speed==180){
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf4096);
  }
  else if(lora_speed==210){
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf4096);
  }
  else if(lora_speed==240){
    rf95.setModemConfig(BG_RF95::Bw125Cr46Sf4096);
  }
  else {
    rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096);
  }
  rf95.setFrequency(lora_FREQ);
  rf95.setTxPower(lora_LTXPower);
  rf95.sendAPRS(lora_TXBUFF, messageSize);
  rf95.waitPacketSent();
  #ifdef ENABLE_LED_SIGNALING
    digitalWrite(TXLED, HIGH);
  #endif
  // cross-digipeating may have altered our RX-frequency. Revert frequency change needed for this transmission.
  if (lora_FREQ != lora_freq)
    rf95.setFrequency(lora_freq);
}

void batt_read(){
#ifdef T_BEAM_V1_0
  BattVolts = axp.getBattVoltage()/1000;
  InpVolts = axp.getVbusVoltage()/1000;
#elif T_BEAM_V0_7
  BattVolts = (((float)analogRead(35) / 8192.0) * 2.0 * 3.3 * (1100.0 / 1000.0))+0.41;    // fixed thanks to Luca IU2FRL 
  //BattVolts = adc1_get_raw(ADC1_CHANNEL_7)/1000;
#else
  BattVolts = analogRead(35)*7.221/4096;
#endif
}

void writedisplaytext(String HeaderTxt, String Line1, String Line2, String Line3, String Line4, String Line5) {
  batt_read();
#ifdef notdef
  if (InpVolts < 1.0) {
    if (BattVolts < 3.5 && BattVolts > 3.3){
      #ifdef T_BEAM_V1_0
        # ifdef ENABLE_LED_SIGNALING
        axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
        #endif
      #endif
    } else if (BattVolts <= 3.3) {
      #ifdef T_BEAM_V1_0
        axp.setChgLEDMode(AXP20X_LED_OFF);
        //axp.shutdown(); <-we need fix this 
        axp.shutdown();
      #endif
    }
  }
  #endif
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println(HeaderTxt);
  display.setTextSize(1);
  display.setCursor(0,16);
  display.println(Line1);
  display.setCursor(0,26);
  display.println(Line2);
  display.setCursor(0,36);
  display.println(Line3);
  display.setCursor(0,46);
  display.println(Line4);
  display.setCursor(0,56);
  display.println(Line5);
  if (!enabled_oled){                         // disable oled
    display.dim(true);
  }   
  display.display();
  time_to_refresh = millis() + showRXTime;
}

String getSatAndBatInfo() {
  String line5;
  if(gps_state == true){
    if(InpVolts > 4){
      line5 = "SAT: " + String(gps.satellites.value()) + "  BAT: " + String(BattVolts, 1) + "V*";
    }else{
      line5 = "SAT: " + String(gps.satellites.value()) + "  BAT: " + String(BattVolts, 2) + "V";
    }
  }else{
    if(InpVolts > 4){
      line5 = "SAT: X  BAT: " + String(BattVolts, 1) + "V*";
    }else{
      line5 = "SAT: X  BAT: " + String(BattVolts, 2) + "V";
    }
    
  }
  #if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
    if (SerialBT.hasClient()){
      line5 += "BT";
    }
  #endif
  return line5;
}

void displayInvalidGPS() {
  char *nextTxInfo;
  if (!gps_state){
    nextTxInfo = (char*)"(TX) GPS DISABLED";
  } else {
    nextTxInfo = (char*)"(TX) at valid GPS";
  }
  writedisplaytext(" " + Tcall, nextTxInfo, "LAT: not valid", "LON: not valid", "SPD: ---  CRS: ---", getSatAndBatInfo());
}

#if defined(KISS_PROTOCOL)
/**
 *
 * @param TNC2FormatedFrame
 */
void sendToTNC(const String& TNC2FormatedFrame) {
  if (tncToSendQueue){
    auto *buffer = new String();
    buffer->concat(TNC2FormatedFrame);
    if (xQueueSend(tncReceivedQueue, &buffer, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      // remove buffer on error
      delete buffer;
    }
  }
}
#endif
#if defined(ENABLE_WIFI)
/**
 *
 * @param TNC2FormatedFrame
 */
void sendToWebList(const String& TNC2FormatedFrame, const int RSSI, const int SNR) {
  if (webListReceivedQueue){
    auto *receivedPacketData = new tReceivedPacketData();
    receivedPacketData->packet = new String();
    receivedPacketData->packet->concat(TNC2FormatedFrame);
    receivedPacketData->RSSI = RSSI;
    receivedPacketData->SNR = SNR;
    getLocalTime(&receivedPacketData->rxTime);

    if (xQueueSend(webListReceivedQueue, &receivedPacketData, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      // remove buffer on error
      delete receivedPacketData->packet;
      delete receivedPacketData;
    }
  }
}
#endif

String prepareCallsign(const String& callsign){
  String tmpString = "";
  for (int i=0; i<callsign.length();++i){  // remove unneeded "spaces" from callsign field
    if (callsign.charAt(i) != ' ') {
      tmpString += callsign.charAt(i);
    }
  }
  return tmpString;
}

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
  void sendTelemetryFrame() {
    if(enable_tel == true){
      #ifdef T_BEAM_V1_0
        uint8_t b_volt = (axp.getBattVoltage() - 3000) / 5.1;
        uint8_t b_in_c = (axp.getBattChargeCurrent()) / 10;
        uint8_t b_out_c = (axp.getBattDischargeCurrent()) / 10;
        uint8_t ac_volt = (axp.getVbusVoltage() - 3000) / 28;
        uint8_t ac_c = (axp.getVbusCurrent()) / 10;
        // Pad telemetry message address to 9 characters
        char Tcall_message_char[9];
        sprintf_P(Tcall_message_char, "%-9s", Tcall.c_str());
        String Tcall_message = String(Tcall_message_char);
        // Flash the light when telemetry is being sent
        #ifdef ENABLE_LED_SIGNALING
          digitalWrite(TXLED, LOW);
        #endif

        // Determine sequence number (or 'MIC')
        String tel_sequence_str;
        if(tel_mic == 1){
          tel_sequence_str = "MIC";
        }else{
          // Get the current saved telemetry sequence
          tel_sequence = preferences.getUInt(PREF_TNC_SELF_TELEMETRY_SEQ, 0);
          // Pad to 3 digits
          char tel_sequence_char[3];
          sprintf_P(tel_sequence_char, "%03u", tel_sequence);
          tel_sequence_str = String(tel_sequence_char);
        }
        // Format telemetry path
        String tel_path_str;
        if(tel_path == ""){
          tel_path_str = tel_path;
        }else{
          tel_path_str = "," + tel_path;
        }

        String telemetryParamsNames = String(":") + Tcall_message + ":PARM.B Volt,B In,B Out,AC V,AC C";
        String telemetryUnitNames = String(":") + Tcall_message + ":UNIT.mV,mA,mA,mV,mA";
        String telemetryEquations = String(":") + Tcall_message + ":EQNS.0,5.1,3000,0,10,0,0,10,0,0,28,3000,0,10,0";
        String telemetryData = String("T#") + tel_sequence_str + "," + String(b_volt) + "," + String(b_in_c) + "," + String(b_out_c) + "," + String(ac_volt) + "," + String(ac_c) + ",00000000";
        String telemetryBase = "";
        telemetryBase += Tcall + ">APLOX1" + tel_path_str + ":";
        Serial.print(telemetryBase);
        sendToTNC(telemetryBase + telemetryParamsNames);
        sendToTNC(telemetryBase + telemetryUnitNames);
        sendToTNC(telemetryBase + telemetryEquations);
        sendToTNC(telemetryBase + telemetryData);

        // Show when telemetry is being sent
        writedisplaytext("((TEL TX))","","","","","");

        // Flash the light when telemetry is being sent
        #ifdef ENABLE_LED_SIGNALING
          digitalWrite(TXLED, HIGH);
        #endif

        // Update the telemetry sequence number
        if(tel_sequence >= 999){
          tel_sequence = 0;
        }else{
          tel_sequence = tel_sequence + 1;
        }
        preferences.putUInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
      #endif
    }  
  }
#endif

// + SETUP --------------------------------------------------------------+//
void setup(){
#ifdef T_BEAM_V0_7 /*
  adcAttachPin(35);
  adcStart(35);
  analogReadResolution(10);
  analogSetAttenuation(ADC_6db); */
  pinMode(35, INPUT);
  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_11);
#endif

  SPI.begin(SPI_sck,SPI_miso,SPI_mosi,SPI_ss);    //DO2JMG Heltec Patch
  Serial.begin(115200);

  #ifdef BUZZER
    ledcSetup(0,1E5,12);
    ledcAttachPin(BUZZER,0);
    ledcWriteTone(0,0);  // turn off buzzer on start
  #endif

  #ifdef DIGI_PATH
    relay_path = DIGI_PATH;
  #else
    relay_path = "";
  #endif

  #ifdef FIXED_BEACON_EN
    fixed_beacon_enabled = true;
  #endif

// This section loads values from saved preferences,
// if available. 
// https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/

  #ifdef ENABLE_PREFERENCES
    int clear_preferences = 0;
    if(digitalRead(BUTTON)==LOW){
      clear_preferences = 1;
    }

    preferences.begin("cfg", false);
    
    // LoRa transmission settings

    if (!preferences.getBool(PREF_LORA_FREQ_PRESET_INIT)){
      preferences.putBool(PREF_LORA_FREQ_PRESET_INIT, true);
      preferences.putDouble(PREF_LORA_FREQ_PRESET, lora_freq);
    }
    lora_freq = preferences.getDouble(PREF_LORA_FREQ_PRESET);
    
    if (!preferences.getBool(PREF_LORA_SPEED_PRESET_INIT)){
      preferences.putBool(PREF_LORA_SPEED_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_SPEED_PRESET, lora_speed);
    }
    lora_speed = preferences.getInt(PREF_LORA_SPEED_PRESET);

    if (!preferences.getBool(PREF_LORA_TX_ENABLE_INIT)){
      preferences.putBool(PREF_LORA_TX_ENABLE_INIT, true);
      preferences.putBool(PREF_LORA_TX_ENABLE, lora_tx_enabled);
    }
    lora_tx_enabled = preferences.getBool(PREF_LORA_TX_ENABLE);

    if (!preferences.getInt(PREF_LORA_TX_POWER_INIT)){
      preferences.putBool(PREF_LORA_TX_POWER_INIT, true);
      preferences.putInt(PREF_LORA_TX_POWER, txPower);
    }
    txPower = lora_tx_enabled ? preferences.getInt(PREF_LORA_TX_POWER) : 0;

    if (!preferences.getBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET_INIT)){
      preferences.putBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET_INIT, true);
      preferences.putBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET, lora_automatic_cr_adaption);
    }
    lora_automatic_cr_adaption = preferences.getBool(PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET);

    if (!preferences.getBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET_INIT)){
      preferences.putBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET_INIT, true);
      preferences.putBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET, lora_add_snr_rssi_to_path);
    }
    lora_add_snr_rssi_to_path = preferences.getBool(PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET);

    if (!preferences.getBool(PREF_APRS_DIGIPEATING_MODE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_DIGIPEATING_MODE_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_DIGIPEATING_MODE_PRESET, lora_digipeating_mode);
    }
    lora_digipeating_mode = preferences.getInt(PREF_APRS_DIGIPEATING_MODE_PRESET);

    if (!preferences.getBool(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET, lora_cross_digipeating_mode);
    }
    lora_cross_digipeating_mode = preferences.getInt(PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET);

    if (!preferences.getBool(PREF_LORA_FREQ_CROSSDIGI_PRESET_INIT)){
      preferences.putBool(PREF_LORA_FREQ_CROSSDIGI_PRESET_INIT, true);
      preferences.putDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET, lora_freq_cross_digi);
    }
    lora_freq_cross_digi = preferences.getDouble(PREF_LORA_FREQ_CROSSDIGI_PRESET);
    

    if (!preferences.getBool(PREF_LORA_SPEED_CROSSDIGI_PRESET_INIT)){
      preferences.putBool(PREF_LORA_SPEED_CROSSDIGI_PRESET_INIT, true);
      preferences.putInt(PREF_LORA_SPEED_CROSSDIGI_PRESET, lora_speed_cross_digi);
    }
    lora_speed_cross_digi = preferences.getInt(PREF_LORA_SPEED_CROSSDIGI_PRESET);

    // APRS station settings

    aprsSymbolTable = preferences.getString(PREF_APRS_SYMBOL_TABLE);
    if (aprsSymbolTable.isEmpty()){
      preferences.putString(PREF_APRS_SYMBOL_TABLE, APRS_SYMBOL_TABLE);
      aprsSymbolTable = preferences.getString(PREF_APRS_SYMBOL_TABLE);
    }

    aprsSymbol = preferences.getString(PREF_APRS_SYMBOL);
    if (aprsSymbol.isEmpty()){
      preferences.putString(PREF_APRS_SYMBOL, APRS_SYMBOL);
      aprsSymbol = preferences.getString(PREF_APRS_SYMBOL, APRS_SYMBOL);
    }

    if (!preferences.getBool(PREF_APRS_COMMENT_INIT)){
      preferences.putBool(PREF_APRS_COMMENT_INIT, true);
      preferences.putString(PREF_APRS_COMMENT, MY_COMMENT);
    }
    aprsComment = preferences.getString(PREF_APRS_COMMENT);

    if (!preferences.getBool(PREF_APRS_RELAY_PATH_INIT)){
      preferences.putBool(PREF_APRS_RELAY_PATH_INIT, true);
      preferences.putString(PREF_APRS_RELAY_PATH, DIGI_PATH);
    }
    relay_path = preferences.getString(PREF_APRS_RELAY_PATH);

    if (!preferences.getBool(PREF_APRS_SHOW_ALTITUDE_INIT)){
      preferences.putBool(PREF_APRS_SHOW_ALTITUDE_INIT, true);
      preferences.putBool(PREF_APRS_SHOW_ALTITUDE, showAltitude);
    }
    showAltitude = preferences.getBool(PREF_APRS_SHOW_ALTITUDE);

    if (!preferences.getBool(PREF_APRS_GPS_EN_INIT)){
      preferences.putBool(PREF_APRS_GPS_EN_INIT, true);
      preferences.putBool(PREF_APRS_GPS_EN, gps_state);
    }
    gps_state = preferences.getBool(PREF_APRS_GPS_EN);

    if (!preferences.getBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS_INIT)){
      preferences.putBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS_INIT, true);
      preferences.putBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS, acceptOwnPositionReportsViaKiss);
    }
    acceptOwnPositionReportsViaKiss = preferences.getBool(PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS);

    if (!preferences.getBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS_INIT)){
      preferences.putBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS_INIT, true);
      preferences.putBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS, gps_allow_sleep_while_kiss);
    }
    gps_allow_sleep_while_kiss = preferences.getBool(PREF_GPS_ALLOW_SLEEP_WHILE_KISS);


    if (!preferences.getBool(PREF_APRS_SHOW_BATTERY_INIT)){
      preferences.putBool(PREF_APRS_SHOW_BATTERY_INIT, true);
      preferences.putBool(PREF_APRS_SHOW_BATTERY, showBattery);
    }
    showBattery = preferences.getBool(PREF_APRS_SHOW_BATTERY);

    if (!preferences.getBool(PREF_ENABLE_TNC_SELF_TELEMETRY_INIT)){
      preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY_INIT, true);
      preferences.putBool(PREF_ENABLE_TNC_SELF_TELEMETRY, enable_tel);
    }
    enable_tel = preferences.getBool(PREF_ENABLE_TNC_SELF_TELEMETRY);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT, true);
      preferences.putInt(PREF_TNC_SELF_TELEMETRY_INTERVAL, tel_interval);
    }
    tel_interval = preferences.getInt(PREF_TNC_SELF_TELEMETRY_INTERVAL);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_SEQ_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_SEQ_INIT, true);
      preferences.putInt(PREF_TNC_SELF_TELEMETRY_SEQ, tel_sequence);
    }
    tel_sequence = preferences.getInt(PREF_TNC_SELF_TELEMETRY_SEQ);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_MIC_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_MIC_INIT, true);
      preferences.putInt(PREF_TNC_SELF_TELEMETRY_MIC, tel_mic);
    }
    tel_mic = preferences.getInt(PREF_TNC_SELF_TELEMETRY_MIC);

    if (!preferences.getBool(PREF_TNC_SELF_TELEMETRY_PATH_INIT)){
      preferences.putBool(PREF_TNC_SELF_TELEMETRY_PATH_INIT, true);
      preferences.putString(PREF_TNC_SELF_TELEMETRY_PATH, tel_path);
    }
    tel_path = preferences.getString(PREF_TNC_SELF_TELEMETRY_PATH);

    if (!preferences.getBool(PREF_APRS_LATITUDE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_LATITUDE_PRESET_INIT, true);
      preferences.putString(PREF_APRS_LATITUDE_PRESET, LATITUDE_PRESET);
    }
    aprsLatPreset = preferences.getString(PREF_APRS_LATITUDE_PRESET);

    if (!preferences.getBool(PREF_APRS_LONGITUDE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_LONGITUDE_PRESET_INIT, true);
      preferences.putString(PREF_APRS_LONGITUDE_PRESET, LONGITUDE_PRESET);
    }
    aprsLonPreset = preferences.getString(PREF_APRS_LONGITUDE_PRESET);

    if (!preferences.getBool(PREF_APRS_FIXED_BEACON_PRESET_INIT)){
      preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET_INIT, true);
      preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET, fixed_beacon_enabled);
    }
    fixed_beacon_enabled = preferences.getBool(PREF_APRS_FIXED_BEACON_PRESET);

    if (!preferences.getBool(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT)){
      preferences.putBool(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET, fix_beacon_interval/1000);
    }
    fix_beacon_interval = preferences.getInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET) * 1000;

// + SMART BEACONING

    if (!preferences.getBool(PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MIN_INTERVAL_PRESET, sb_min_interval/1000);
    }
    sb_min_interval = preferences.getInt(PREF_APRS_SB_MIN_INTERVAL_PRESET) * 1000;
    if (sb_min_interval < 10000) sb_min_interval = 10000;

    if (!preferences.getBool(PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MAX_INTERVAL_PRESET, sb_max_interval/1000);
    }
    sb_max_interval = preferences.getInt(PREF_APRS_SB_MAX_INTERVAL_PRESET) * 1000;
    if (sb_max_interval <= sb_min_interval) sb_max_interval = sb_min_interval + 1000;

    if (!preferences.getBool(PREF_APRS_SB_MIN_SPEED_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MIN_SPEED_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MIN_SPEED_PRESET, sb_min_speed);
    }
    sb_min_speed = (float) preferences.getInt(PREF_APRS_SB_MIN_SPEED_PRESET);
    if (sb_min_speed < 0) sb_min_speed = 0;

    if (!preferences.getBool(PREF_APRS_SB_MAX_SPEED_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_MAX_SPEED_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_MAX_SPEED_PRESET, sb_max_speed);
    }
    sb_max_speed = (float ) preferences.getInt(PREF_APRS_SB_MAX_SPEED_PRESET);
    if (sb_max_speed <= sb_min_speed) sb_max_speed = sb_min_speed +1;

    if (!preferences.getBool(PREF_APRS_SB_ANGLE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_ANGLE_PRESET_INIT, true);
      preferences.putDouble(PREF_APRS_SB_ANGLE_PRESET, sb_angle);
    }
    sb_angle = preferences.getDouble(PREF_APRS_SB_ANGLE_PRESET);

    if (!preferences.getBool(PREF_APRS_SB_TURN_SLOPE_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_TURN_SLOPE_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_TURN_SLOPE_PRESET, sb_turn_slope);
    }
    sb_turn_slope = preferences.getInt(PREF_APRS_SB_TURN_SLOPE_PRESET);

    if (!preferences.getBool(PREF_APRS_SB_TURN_TIME_PRESET_INIT)){
      preferences.putBool(PREF_APRS_SB_TURN_TIME_PRESET_INIT, true);
      preferences.putInt(PREF_APRS_SB_TURN_TIME_PRESET, sb_turn_time);
    }
    sb_turn_time = preferences.getInt(PREF_APRS_SB_TURN_TIME_PRESET);

// 

    if (!preferences.getBool(PREF_DEV_SHOW_RX_TIME_INIT)){
      preferences.putBool(PREF_DEV_SHOW_RX_TIME_INIT, true);
      preferences.putInt(PREF_DEV_SHOW_RX_TIME, showRXTime/1000);
    }
    showRXTime = preferences.getInt(PREF_DEV_SHOW_RX_TIME) * 1000;

    // Read OLED RX Timer
    if (!preferences.getBool(PREF_DEV_SHOW_OLED_TIME_INIT)){
      preferences.putBool(PREF_DEV_SHOW_OLED_TIME_INIT, true);
      preferences.putInt(PREF_DEV_SHOW_OLED_TIME, oled_timeout/1000);
    }
    oled_timeout = preferences.getInt(PREF_DEV_SHOW_OLED_TIME) * 1000;
    
    if (!preferences.getBool(PREF_DEV_AUTO_SHUT_PRESET_INIT)){
      preferences.putBool(PREF_DEV_AUTO_SHUT_PRESET_INIT, true);
      preferences.putInt(PREF_DEV_AUTO_SHUT_PRESET, shutdown_delay_time/1000);
    }
    shutdown_delay_time = preferences.getInt(PREF_DEV_AUTO_SHUT_PRESET) * 1000;

    if (!preferences.getBool(PREF_DEV_AUTO_SHUT_INIT)){
      preferences.putBool(PREF_DEV_AUTO_SHUT_INIT, true);
      preferences.putBool(PREF_DEV_AUTO_SHUT, shutdown_active);
    }
    shutdown_active = preferences.getBool(PREF_DEV_AUTO_SHUT);          

    if (clear_preferences){
      delay(1000);
      if(digitalRead(BUTTON)==LOW){
        clear_preferences = 2;
      }
    }

    if (!preferences.getBool(PREF_APRS_SHOW_CMT_INIT)){
      preferences.putBool(PREF_APRS_SHOW_CMT_INIT, true);
      preferences.putBool(PREF_APRS_SHOW_CMT, show_cmt);
    }
    show_cmt = preferences.getBool(PREF_APRS_SHOW_CMT);

    if (!preferences.getBool(PREF_APRS_COMMENT_RATELIMIT_PRESET_INIT)){
      preferences.putBool(PREF_APRS_COMMENT_RATELIMIT_PRESET_INIT, true);
      preferences.putBool(PREF_APRS_COMMENT_RATELIMIT_PRESET, rate_limit_message_text);
    }
    rate_limit_message_text = preferences.getBool(PREF_APRS_COMMENT_RATELIMIT_PRESET);


    if (!preferences.getBool(PREF_DEV_BT_EN_INIT)){
      preferences.putBool(PREF_DEV_BT_EN_INIT, true);
      preferences.putBool(PREF_DEV_BT_EN, enable_bluetooth);
    }
    enable_bluetooth = preferences.getBool(PREF_DEV_BT_EN);

    if (!preferences.getBool(PREF_DEV_OL_EN_INIT)){
      preferences.putBool(PREF_DEV_OL_EN_INIT, true);
      preferences.putBool(PREF_DEV_OL_EN,enabled_oled);
    }
    enabled_oled  = preferences.getBool(PREF_DEV_OL_EN); 
  #endif

// enforce valid transmissions even on wrong configurations
if (aprsSymbolTable.length() != 1)
  aprsSymbolTable = String("/");
if (aprsSymbol.length() != 1)
  aprsSymbol = String("[");
if (aprsLatPreset.length() != 8 || !(aprsLatPreset.endsWith("N") || aprsLatPreset.endsWith("S")) || aprsLatPreset.c_str()[4] != '.')
  aprsLatPreset = String("0000.00N");
if (aprsLonPreset.length() != 9 || !(aprsLonPreset.endsWith("E") || aprsLonPreset.endsWith("W")) || aprsLonPreset.c_str()[5] != '.')
  aprsLonPreset = String("00000.00E");

  for (int i=0;i<ANGLE_AVGS;i++) {                                        // set average_course to "0"
    average_course[i]=0;
  }

  pinMode(TXLED, OUTPUT);
  #ifdef T_BEAM_V1_0
    pinMode(BUTTON, INPUT);
  #elif T_BEAM_V0_7
    pinMode(BUTTON, INPUT);
  #else
    pinMode(BUTTON, INPUT_PULLUP);
  #endif
  digitalWrite(TXLED, LOW);                                               // turn blue LED off
  
  Wire.begin(I2C_SDA, I2C_SCL);

  #ifdef T_BEAM_V1_0
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    }
    axp.setLowTemp(0xFF);                                                 //SP6VWX Set low charging temperature
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);                           // LoRa
    if (gps_state){
      axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);                           // switch on GPS
    } else {
      axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);                           // switch off GPS
    }
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setDCDC1Voltage(3300);
    // Enable ADC to measure battery current, USB voltage etc.
    axp.adc1Enable(0xfe, true);
    axp.adc2Enable(0x80, true);
    axp.setChgLEDMode(AXP20X_LED_OFF);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);                          // oled do not turn off     
  #endif

  if(!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDRESS)) {
      for(;;);                                                             // Don't proceed, loop forever
  }

  #ifdef ENABLE_PREFERENCES
    if (clear_preferences == 2){
      writedisplaytext("LoRa-APRS","","","Factory reset","","");
      delay(1000);
      //#ifdef T_BEAM_V1_0
        if(digitalRead(BUTTON)==LOW){
          clear_preferences = 3;
          preferences.clear();
          preferences.end();
          writedisplaytext("LoRa-APRS","","Factory reset","Done!","","");
          delay(2000);
          ESP.restart();
        } else {
          writedisplaytext("LoRa-APRS","","Factory reset","Cancel","","");
          delay(2000);
        }
      //#endif
    }
  #endif
  writedisplaytext("LoRa-APRS","","Init:","Display OK!","","");

  Tcall = prepareCallsign(String(CALLSIGN));
  #ifdef ENABLE_PREFERENCES
    Tcall = preferences.getString(PREF_APRS_CALLSIGN);
    if (Tcall.isEmpty()){
      preferences.putString(PREF_APRS_CALLSIGN, String(CALLSIGN));
      Tcall = preferences.getString(PREF_APRS_CALLSIGN);
    }
  #endif

  if (!rf95.init()) {
    writedisplaytext("LoRa-APRS","","Init:","RF95 FAILED!",":-(","");
    for(;;); // Don't proceed, loop forever
  }

  if (sb_max_interval < nextTX){
    sb_max_interval=nextTX;
  }
  writedisplaytext("LoRa-APRS","","Init:","RF95 OK!","","");
  writedisplaytext(" "+Tcall,"","Init:","Waiting for GPS","","");
  xTaskCreate(taskGPS, "taskGPS", 5000, nullptr, 1, nullptr);
  writedisplaytext(" "+Tcall,"","Init:","GPS Task Created!","","");
  #ifndef T_BEAM_V1_0
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  #endif
  batt_read();
  writedisplaytext("LoRa-APRS","","Init:","ADC OK!","BAT: "+String(BattVolts,2),"");
  
  if(lora_speed==1200){
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf512);
  }
  else if(lora_speed==610){
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf1024);
  }
  else if(lora_speed==180){
    rf95.setModemConfig(BG_RF95::Bw125Cr48Sf4096);
  }
  else if(lora_speed==210){
    rf95.setModemConfig(BG_RF95::Bw125Cr47Sf4096);
  }
  else if(lora_speed==240){
    rf95.setModemConfig(BG_RF95::Bw125Cr46Sf4096);
  }
  else {
    rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096);
  }

  Serial.printf("LoRa Speed:\t%lu\n", lora_speed);
  
  rf95.setFrequency(lora_freq);
  Serial.printf("LoRa FREQ:\t%f\n", lora_freq);
  rf95.setTxPower(txPower);
  delay(250);
  #ifdef KISS_PROTOCOL
    xTaskCreatePinnedToCore(taskTNC, "taskTNC", 10000, nullptr, 1, nullptr, xPortGetCoreID());
  #endif

  #if defined(KISS_PROTOCOL)
    if (enable_bluetooth){
      #ifdef BLUETOOTH_PIN
        SerialBT.setPin(BLUETOOTH_PIN);
      #endif
      #ifdef ENABLE_BLUETOOTH
        SerialBT.begin(String("TTGO LORA APRS ") + Tcall);
        writedisplaytext("LoRa-APRS","","Init:","BT OK!","","");
      #endif
    }
  #endif

  #ifdef ENABLE_WIFI
    webServerCfg = {.callsign = Tcall};
    xTaskCreate(taskWebServer, "taskWebServer", 12000, (void*)(&webServerCfg), 1, nullptr);
    writedisplaytext("LoRa-APRS","","Init:","WiFi task started","   =:-)   ","");
  #endif

  writedisplaytext("LoRa-APRS","","Init:","FINISHED OK!","   =:-)   ","");
  writedisplaytext("","","","","","");
  time_to_refresh = millis() + showRXTime;
  displayInvalidGPS();
  digitalWrite(TXLED, HIGH);

  // Hold the OLED ON at first boot
  oled_timer=millis()+oled_timeout;
}

void enableOled() {
  // This function enables OLED display after pressing a button
  tempOled = true;
  oled_timer = millis() + oled_timeout;
}


// rf95.lastSNR() returns unsigned 8bit value, which we get as input
int bg_rf95snr_to_snr(uint8_t snr)
{
    // SNR values reported by rf95.lastSNR() are not plausible. See those two projects:
    // https://github.com/Lora-net/LoRaMac-node/issues/275
    // https://github.com/mayeranalytics/pySX127x/blob/master/SX127x/LoRa.py
    // We use an old BG_RF95 library from 2001. RadioHead/RH_RF95.cpp implements _lastSNR and _lastRssi correctly accordingg to the specs
    // Per page 111, SX1276/77/78/79 datasheet
    //return ( ( snr > 127 ) ? (snr - 256) : snr ) / 4.0;
    if (snr & 0x80) {
      return -(( ( ~snr + 1 ) & 0xFF ) >> 2);
    }
    return ( snr & 0x7F ) >> 2;
}


int bg_rf95rssi_to_rssi(int rssi)
{
  // We use an old BG_RF95 library from 2001. RadioHead/RH_RF95.cpp implements _lastSNR and _lastRssi correctly accordingg to the specs
  int _lastSNR = bg_rf95snr_to_snr(rf95.lastSNR());
  boolean _usingHFport = (lora_freq >= 779.0);

  // bg_rf95 library: _lastRssi = spiRead(BG_RF95_REG_1A_PKT_RSSI_VALUE) - 137. First, undo -137 ooperation
  int _lastRssi = rf95.lastRssi() + 137;

  if (_lastSNR < 0)
    _lastRssi = _lastRssi + _lastSNR;
  else
    _lastRssi = (int)_lastRssi * 16 / 15;
  if (_usingHFport)
    _lastRssi -= 157;
  else
     _lastRssi -= 164;
  return _lastRssi;
}


char *encode_snr_rssi_in_path()
{
  static char buf[7]; // length for "Q2373X" == 6 + 1 (\0) == 7
  *buf = 0;
  if (lora_add_snr_rssi_to_path) {
    // SNR values reported by rf95.lastSNR() are not plausible. See those two projects:
    // https://github.com/Lora-net/LoRaMac-node/issues/275
    // https://github.com/mayeranalytics/pySX127x/blob/master/SX127x/LoRa.py
    // rf95snr_to_snr returns values in range -32 to 31. The lowest two bits are RFU
    int snr = bg_rf95snr_to_snr(rf95.lastSNR());
    int rssi = bg_rf95rssi_to_rssi(rf95.lastRssi());
    if (snr > 99) snr = 99;			// snr will not go upper 31
    else if (snr < -99) snr = -99;		// snr will not go below -32
    if (rssi > 0) rssi = 0;			// rssi is always negative
    else if (rssi < -259) rssi = -259;		// rssi will not be below -174 anyway ;)
    // Make SNR >= 0 human readable:
    //   First position: SNR < 0: replace -1 by A1, -10 by B0, ...
    //   This way, we reduce "-10" to two letters: B0.
    // Make RSSI  > -100 RSSI human readable.
    //   First position: rssi < -99 -> 0. We'll use 91 for rssi -91 instead of J1 for better readibility.
    //   K means -10x, L means -11x, M means -12x, N means -13x, ...   (everone knows, 'N' is #13 in alphabet out of 26 chars ;)
    //   => With this, we reduce "-110" to two letters: L0.
    // Last position of call (6) is left empty for future use. As well as SSID 1-15. Could be used for BER, RX antenna gain, EIRP, ..
    // => This is a good compromise between efficiency and being able to quickly interprete snr and rssi
    sprintf(buf, "Q%c%01d%c%01d",
      ((snr >= 0 ? snr : -snr) / 10) + (snr >= 0 ? '0' : 'A'),
      (snr >= 0 ? snr : -snr) % 10,
      (-rssi / 10) + (rssi > -100 ? '0' : 'A'),
      (-rssi) % 10);
  }
  return buf;
}


char *add_element_to_path(const char *data, const char *element)
{
  static char buf[BG_RF95_MAX_MESSAGE_LEN+1];
  if (strlen(data) + 1 /* ',' */ + strlen(element) > sizeof(buf)-1)
    return 0;
  char *p = strchr(data, '>');
  char *header_end = strchr(data, ':');
  if (header_end <= p)
    return 0;
  char *q = strchr(data, ',');
  if (q > header_end)
    q = 0;
  if (q && q < p)
    return 0;
  if (q) {
    int n_digis = 1;
    char *n = q;
    for (;;) {
      n = strchr(n+1, ',');
      if (!n || n > header_end)
        break;
      n_digis++;
    }
  if (n_digis > 7)
    return 0;
  }

  char *r = 0;
  if (q && (r = strchr(q+1, '*')) > 0 && r < header_end) {
    // behind last digipeated call
    for (;;) {
      char *rr = strchr(r+1, '*');
      if (!rr || r > header_end)
        break;
      r = rr;
    }
  }
  char *pos = header_end;
  if (q) {
    // Something like DL9SAU>APRS,NOGATE -> DL9SAU>APRS,MYCALL*,NOGATE DL9SAU>APRS,DL1AAA*,WIDE2-1 -> DL9SAU>APRS,DL1AAA,MYCALL*,WIDE2-1
    // r points to '*', r+1 points to ',' and we omit '*'; q points to ','
    pos = r ? r : q;
  } // else: Something like DL9SAU>APRS:... (no via path)
  snprintf(buf, pos-data +1, "%s", data);
  sprintf(buf + strlen(buf), ",%s*%s", element, pos + (*pos == '*' ? 1 : 0));
  return buf;
}


#define AX_ADDR_LEN 9   // room for "DL9SAU-15" == 9
#define AX_DIGIS_MAX 8

struct axaddr {
  char addr[AX_ADDR_LEN+1];
  boolean repeated;
};

struct ax25_frame {
  struct axaddr src;
  struct axaddr dst;
  struct axaddr digis[AX_DIGIS_MAX];
  uint8_t n_digis;
  char *data;
};

struct ax25_frame *tnc_format_to_ax25_frame(const char *s)
{
  static char data[BG_RF95_MAX_MESSAGE_LEN+1];
  static struct ax25_frame frame;
  char *p;
  char *q;
  char *next_digi = 0;

  memset(&frame, 0, sizeof(frame));
  *data = 0;

  if (strlen(s) > sizeof(data) -1)
    return 0;
  strcpy(data, s);

  p = strchr(data, ':');
  if (!p || strlen(p) > sizeof(data)-1)
    return 0;
  // skip leading ':'
  frame.data = p+1;
  *p = 0;

  p = data;
  if (!(q = strchr(p, '>')))
    return 0;
  *q = 0;
  if (!*p || strlen(p) > AX_ADDR_LEN)
    return 0;
  strcpy(frame.src.addr, p);

  p = q+1;
  if ((q = strchr(p, ','))) {
    next_digi = q+1;
    *q = 0;
  }
  if (!*p || strlen(p) > AX_ADDR_LEN)
    return 0;
  strcpy(frame.dst.addr, p);

  int digi;
  for (digi = 0; digi < AX_DIGIS_MAX && next_digi; digi++) {
    p = next_digi;
    if ((q = strchr(p, ','))) {
      next_digi = q+1;
      *q = 0;
    } else
      next_digi = 0;

    if ((q = strchr(p, '*'))) {
      frame.digis[digi].repeated = true;
      *q = 0;
    }
    if (!*p || strlen(p) > AX_ADDR_LEN)
      return 0;
    strcpy(frame.digis[digi].addr, p);
    // max digi cound reached, but frame had more digis in path
    if (digi == AX_DIGIS_MAX-1 && next_digi)
      return 0;
  }

  // sanity check
  for (digi = 0; digi < AX_DIGIS_MAX && *(frame.digis[digi].addr); digi++) {
    if (!frame.digis[digi].repeated && !strncmp(frame.digis[digi].addr, "WIDE", 4) && strlen(frame.digis[digi].addr) == 5)
        frame.digis[digi].repeated = true;
  }
  boolean last_repeated_found = false;
  for (digi = AX_DIGIS_MAX-1; digi >= 0; digi--) {
    if (!*(frame.digis[digi].addr))
      continue;
    // once: set n_digis
    if (!frame.n_digis)
      frame.n_digis = digi+1;
    if (!last_repeated_found && frame.digis[digi].repeated)
      last_repeated_found = true;
    else if (last_repeated_found && !frame.digis[digi].repeated)
      frame.digis[digi].repeated = true;
  }

  return &frame;
}


void handle_lora_frame_for_lora_digipeating(const char *received_frame, char *snr_rssi)
{

  struct ax25_frame* frame = tnc_format_to_ax25_frame(received_frame);

  if (!frame)
    return;

  // no room left for adding our call in path during repeating
  if (frame->n_digis > AX_DIGIS_MAX)
    return;

  // '>' and ':' found. Always in header. Sanity check: if ',' present, it must be > p. If r exists, must be > q. And header_end must be > than the others and always a message-body *(header_end + 1) != 0.

  // wide1-digi case
  if (lora_digipeating_mode == 2) {
    const char *p = strchr(received_frame, '>');
    const char *q = strchr(received_frame, ','); // digis, optional 
    const char *r = strchr(received_frame, '*');
    const char *header_end = strchr(received_frame, ':');
    // we hear an packet, digipeated from another digipeater (same source call, digipeated flag '*' in header)
    // and have the original packet in our digipeating queue? -> clear queue.
    // If we are a WIDE2 digi, it may be desired that we digipeat him.
    // We'll throw that frame away if further down the new frame is worth digipeating. We don't build up Digipeating-TX-queues
    if (p && strncmp(lora_TXBUFF_for_digipeating, received_frame, p-received_frame) && r && r > q && r < header_end && *(header_end+1))
      *lora_TXBUFF_for_digipeating = 0;
  }

  // src-call_validation
  char *p_call = frame->src.addr;
  int i = 0;
  if (isdigit(p_call[1]) && isalpha(p_call[2])) {
    // left-shift g1abc -> _g1abc
    i = 1;
    p_call--; // warning, beyond start of pointer
  }
  for (; i <= 7; i++) {
    if (i == 7) return;
    else if (!p_call[i] || p_call[i] == '-') {
      if (i < 4) return;
      break;
    } else if (i < 2 && !isalnum(p_call[i])) return;
    else if (i == 2 && !isdigit(p_call[i])) return;
    else if (i > 2 && !isalpha(p_call[i])) return;
  }

  // If DST-call-ssid-digipeating: rewrite before adding path, because WIDE in path and DST-SSID-digipeating are mutual exclusive
  char *q;
  if ((q = strchr(frame->dst.addr, '-'))) {
    if (frame->n_digis && frame->digis[0].repeated)
      return;
    if (frame->n_digis > AX_DIGIS_MAX-1)
      return;
    *q++ = 0;
    if (*q < '1' || *q > '7' || q[1] )
      return;
    for (i = 0; i < frame->n_digis; i++)
      if (!strncmp(frame->digis[i].addr, "WIDE", 4) || !strcmp(frame->digis[i].addr, Tcall.c_str()))
        return;
    // move digi path one right
    for (i = frame->n_digis-1; i >= 0; i--) {
      strcpy(frame->digis[i+1].addr, frame->digis[i].addr);
      frame->digis[i+1].repeated = frame->digis[i].repeated;
    }
    sprintf(frame->digis[0].addr, "WIDE%c-%c", *q, *q);
    frame->digis[0].repeated = false;
    frame->n_digis++;
  }

  // nothing to repeat:
  if (!frame->n_digis)
    return;

  // we do not digipeat our own packets
  if (!strcmp(frame->src.addr, Tcall.c_str()))
    return;

  int curr_not_repeated = 0;
  for (i = 0; i < frame->n_digis; i++) {
    if (!frame->digis[i].repeated) {
      curr_not_repeated = i;
      break;
    }
    // our call with digipeated marker in header?
    if (!strcmp(frame->digis[i].addr, Tcall.c_str()))
      return;
  }

  // our call in path?
  boolean add_our_call = true;
  int insert_our_data_before = -1;
  if (!strcmp(frame->digis[curr_not_repeated].addr, Tcall.c_str())) {
    // digi path too long for adding snr_rssi? skip adding snr_rssi
    if (*snr_rssi && frame->n_digis == AX_DIGIS_MAX)
      *snr_rssi = 0;
    frame->digis[curr_not_repeated].repeated = true;
    add_our_call = false;
    insert_our_data_before = curr_not_repeated;
    // if we are a digicall-only digi, our job ends here
    goto add_our_data;
  }

  // digi path too long for adding our call skip adding snr_rssi
  if (frame->n_digis == AX_DIGIS_MAX)
    return;
  // digi path too long for adding snr_rssi and our call? skip adding snr_rssi
  if (frame->n_digis +1 == AX_DIGIS_MAX)
    *snr_rssi = 0;

  if (!strcmp(frame->digis[curr_not_repeated].addr, "WIDE1-1")) {
    frame->digis[curr_not_repeated].addr[5] = 0;
    frame->digis[curr_not_repeated].repeated = 1;
    if (insert_our_data_before < 0)
      insert_our_data_before = curr_not_repeated;
    curr_not_repeated++;
    // If we are a WIDE2 digi, we'll also change WIDE2-n to WIDE2* further down
  }
  if (lora_digipeating_mode < 3)
    goto add_our_data;

  if (!strncmp(frame->digis[curr_not_repeated].addr, "WIDE", 4) && strlen(frame->digis[curr_not_repeated].addr) == 7) {
    if (frame->digis[curr_not_repeated].addr[4] < '2' || frame->digis[curr_not_repeated].addr[4] > '3')
      return;
    if (frame->digis[curr_not_repeated].addr[5] != '-')
      return;
    // bad syntax?
    if (frame->digis[curr_not_repeated].addr[6] < '1' || frame->digis[curr_not_repeated].addr[6] > frame->digis[curr_not_repeated].addr[4])
      return;
    // prevent abuse
    if (curr_not_repeated+1 < frame->n_digis && !strncmp(frame->digis[curr_not_repeated+1].addr, "WIDE", 4))
        return;
    // mark every WIDEn-m as WIDEn and repeated
    frame->digis[curr_not_repeated].addr[5] = 0;
    frame->digis[curr_not_repeated].repeated = 1;
    if (insert_our_data_before < 0)
      insert_our_data_before = curr_not_repeated;
  }

add_our_data:

  // insert_our_data still < 0? i.e. searches like that for WIDE2-1 in path did not lead to a result
  if (insert_our_data_before < 0)
    return;


  // Build txbuff:

  char buf[sizeof(lora_TXBUFF_for_digipeating)];

  sprintf(buf, "%s>%s", frame->src.addr, frame->dst.addr);
  char *digi_paths_start = buf + strlen(buf);

  for (i = 0; i < insert_our_data_before; i++)
    sprintf(buf + strlen(buf), ",%s", frame->digis[i].addr);

  if (*snr_rssi)
    sprintf(buf + strlen(buf), ",%s", snr_rssi);

  if (add_our_call)
    sprintf(buf + strlen(buf), ",%s", Tcall.c_str());

  for (i = insert_our_data_before; i < frame->n_digis; i++) {
    sprintf(buf + strlen(buf), ",%s", frame->digis[i].addr);
    if (frame->digis[i].repeated) {
      i++;
      break;
    }
  }

  // we have something added after "SRC>DST"?
  if (strlen(buf) > (digi_paths_start-buf))
    strcat(buf, "*");
  for (; i < frame->n_digis; i++)
    sprintf(buf + strlen(buf), ",%s", frame->digis[i].addr);

  // length check:
  if (strlen(buf) + 1 /* : */ + strlen(frame->data) > sizeof(lora_TXBUFF_for_digipeating)-1)
    return;

  sprintf(lora_TXBUFF_for_digipeating, "%s:%s", buf, frame->data);
  time_lora_TXBUFF_for_digipeating_was_filled = millis();

}


// +---------------------------------------------------------------------+//
// + MAINLOOP -----------------------------------------------------------+//
// +---------------------------------------------------------------------+//
void loop() {
  if(digitalRead(BUTTON)==LOW && key_up == true){
    key_up = false;
    delay(50);
    if(digitalRead(BUTTON)==LOW){
      delay(300);
      time_delay = millis() + 1500;
      if(digitalRead(BUTTON)==HIGH){
        if (!tempOled && enabled_oled) {
        enableOled(); // turn ON OLED temporary
        } else {
          if(gps_state == true && gps.location.isValid()){
              writedisplaytext("((MAN TX))","","","","","");
              sendpacket();
          }else{
              writedisplaytext("((FIX TX))","","","","","");
              sendpacket();
          }
        }
        key_up = true;
      }
    }
  }

  // Show informations on WiFi Status
  if (apConnected) {
    enableOled(); // turn ON OLED temporary
    writedisplaytext(" ((WiFi))","WiFi Client Mode","SSID: " + infoApName, "Pass: ********", "IP: " + infoApAddr, getSatAndBatInfo());
    apConnected=false;
  } else if (apEnabled) {
    enableOled(); // turn ON OLED temporary
    writedisplaytext(" ((WiFi))","WiFi AP Mode","SSID: " + infoApName, "Pass: " + infoApPass, "IP: " + infoApAddr, getSatAndBatInfo());
    apEnabled=false;
  }

  if (manBeacon) {
    // Manually sending beacon from html page
    enableOled();
    writedisplaytext("((WEB TX))","","","","","");
    sendpacket();
    manBeacon=false;
  }
  // Only wake up OLED when necessary, note that DIM is to turn OFF the backlight
  if (enabled_oled) {
    if (oled_timeout>0) {
      display.dim(!tempOled);
    } else {
      // If timeout is 0 keep OLED awake
      display.dim(false);
    }
  } 

  if (tempOled && millis()>= oled_timer) {
    tempOled = false; // After some time reset backlight
  }

  if(digitalRead(BUTTON)==LOW && key_up == false && millis() >= time_delay && t_lock == false){
    // enable OLED
    enableOled();
    //---------------
    t_lock = true;
      if(gps_state){
        gps_state = false;
        #ifdef T_BEAM_V1_0
          axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);                 // GPS OFF
        #endif
        writedisplaytext("((GPSOFF))","","","","","");
        next_fixed_beacon = millis() + fix_beacon_interval;
        #ifdef ENABLE_PREFERENCES
          preferences.putBool(PREF_APRS_GPS_EN, false);
        #endif
      }else{
        gps_state = true;
        #ifdef T_BEAM_V1_0
          axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
        #endif
        writedisplaytext("((GPS ON))","","","","","");                // GPS ON
        #ifdef ENABLE_PREFERENCES
          preferences.putBool(PREF_APRS_GPS_EN, true);
        #endif
      }
  }
  
  if(digitalRead(BUTTON)==HIGH && !key_up){
    key_up = true;
    t_lock = false;
  }

  if (dont_send_own_position_packets) {
#ifdef KISS_PROTOCOL
    // reset to default state if kiss device is disconnected, silent or last position frame was seen long time ago (i.e. 1h).
    // TODO: also iterate through KISS tcp-devices in the parts below wih 'kiss_client_came_via_bluetooth'
    if (
#ifdef ENABLE_BLUETOOTH
         (kiss_client_came_via_bluetooth && !SerialBT.hasClient()) ||
#endif
         (((time_last_own_position_via_kiss_received + sb_max_interval + 10*1000L) < millis()) &&
             time_last_own_position_via_kiss_received >= time_last_frame_via_kiss_received) ||
           // ^kiss client has not recently sent a position gain (sb_max_interval plus 10 seconds grace) and kiss client sent no other data
	 ((time_last_frame_via_kiss_received + sb_max_interval * 2 + 10*1000L) < millis())) {
            // ^ kiss client sent no positions and stoped sending other data for 2*sb_max_interval (plus 10 seconds grace)
#ifdef T_BEAM_V1_0
      if (!gps_state && gps_state_before_autochange)
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
#endif
      gps_state = gps_state_before_autochange;
      dont_send_own_position_packets = false;
      time_last_own_position_via_kiss_received = 0L;
      time_last_frame_via_kiss_received = 0L;
#ifdef ENABLE_BLUETOOTH
      if (kiss_client_came_via_bluetooth && !SerialBT.hasClient())
        kiss_client_came_via_bluetooth = false;
#endif
    }
#endif
  }
  if (fixed_beacon_enabled && !dont_send_own_position_packets && lora_tx_enabled) {
    if (millis() >= next_fixed_beacon && !gps_state) {
      enableOled(); // enable OLED
      next_fixed_beacon = millis() + fix_beacon_interval;
      writedisplaytext("((AUT TX))", "", "", "", "", "");
      sendpacket();
    }
  }

  #ifdef T_BEAM_V1_0
    if(shutdown_active){
      if(InpVolts> 4){
        shutdown_usb_status_bef = true;
        shutdown_countdown_timer_enable = false;
      }

      if(InpVolts < 4 && shutdown_usb_status_bef == true){
        shutdown_usb_status_bef = false;
        shutdown_countdown_timer_enable = true;
        shutdown_countdown_timer = millis() + shutdown_delay_time;
      }

      if(shutdown_countdown_timer_enable){
        if(millis() >= shutdown_countdown_timer){
          axp.setChgLEDMode(AXP20X_LED_OFF);
          axp.shutdown();
        }
      }
    }
  #endif

  #ifdef KISS_PROTOCOL
    String *TNC2DataFrame = nullptr;
    if (tncToSendQueue) {
      if (xQueueReceive(tncToSendQueue, &TNC2DataFrame, (1 / portTICK_PERIOD_MS)) == pdPASS) {
        time_last_frame_via_kiss_received = millis();
        const char *data = TNC2DataFrame->c_str();
	// Frame comes from same call as ours and is a position report?
        if (!strncmp(data, Tcall.c_str(), Tcall.length()) && data[Tcall.length()] == '>') {
	  char *p = strchr(data, ':');
	  p++;
	  if (*p == '!' || *p == '=' || *p == '/' || *p == '@' || *p == '\'' || *p == '`' || *p == '[' || *p == '$') {
            time_last_own_position_via_kiss_received = time_last_frame_via_kiss_received;
	    if (!acceptOwnPositionReportsViaKiss)
	      goto out; // throw away this frame.
            if (!dont_send_own_position_packets) {
	      gps_state_before_autochange = gps_state;
	      if (gps_allow_sleep_while_kiss) {
#ifdef T_BEAM_V1_0
	        if (gps_state)
	          axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);                           // switch off GPS
#endif
	        gps_state = false;
	      }
	      dont_send_own_position_packets = true;
	      // TODO: there are also tcp kiss devices. Instead of 'kiss_client_came_via_bluetooth', we should mark it in a session struct where we can iterate through
#ifdef ENABLE_BLUETOOTH
	      if (SerialBT.hasClient())
	        kiss_client_came_via_bluetooth = true;
#endif
	    }
	  } else if (*p == ':')
	    time_last_own_text_message_via_kiss_received = millis();
        } else {
	  if (lora_digipeating_mode > 1) {
            const char *p = strchr(data, '>');
            const char *q = strchr(data, ',');
            const char *r = strchr(data, '*');
            const char *header_end = strchr(data, ':');
	    // packet (sender not our call) came via kiss and we gate it to lora? If we are a configured as a digipeater,
	    // and our gated packet will be repeated, then we have avoid repeating it again, by adding our call to the digi path.
	    // Example: DL9SAU>APRS,WIDE1-1,WIDE2-1. If we gate it and a fill-in-digi hears it, if becomes DL9SAU>APRS,DL1AAA,WIDE1*,WIDE2-1.
	    // If we are a wide-digi, we see that packet and don't find our callsign in the path, we would repeat it.
	    // => We should add our callsign to path, with the following exceptions:
	    //   1. if sender is our call (already checked above);
	    //   2. if we are not a digipeater and user has a software like aprsdroid and sends packets with another ssid (-> even we are not in the
	    // path, there's no risk, we will not repeat it - as said, we are not configured as a repeater).
	    //   3. DEST-call-addressing with one hop ("DST-1")
	    //   4. In case of WIDE1 or WIDE2-1 (== with one hop left).
	    // We assure bufffer has enough room for our call ",CALL*"
	    if (q > header_end) q = 0;
	    if (r > header_end) r = 0;
	    if (p && header_end > p) {
	      boolean add_our_call = true;
	      // Has the path only one hop or less left (WIDE1-1 or WIDEn or WiDEn* or WIDEn*,WIDEn-1 or WIDEn-1)?
	      // Also check for digicall,WIDE1-1;  digicall*,WIDE1-1 is ok.   Pos ',' before wide is != q (q is first ',' in ..>APRS,DIGICALL,WIDE2-1)
	      char *wide_hop = strstr(data, ",WIDE");
	      if (wide_hop && wide_hop < header_end && wide_hop != q && *(wide_hop-1) != '*')
	        add_our_call = false;
	      if (add_our_call) {
	        for (;;) { 
	          if (!wide_hop || wide_hop > header_end) {
		    wide_hop = 0;
		    break;
		  }
	          // pos 5 is number, pos 6 is '-', pos 7 are left digis
	          if (wide_hop[6] == '-' && wide_hop[7] != '1')
	            break;
		  wide_hop = strstr(wide_hop+8, ",WIDE");
                } 
	      }
	      if (!wide_hop)
	        add_our_call = false;
	      else { 
	        // Is our callsign in the path? -> Fine. But only if repeated-bit is set.
	        char *our_call = strstr(p+1, Tcall.c_str());
	        if (our_call && our_call < header_end && *(our_call-1) == ',' && our_call[Tcall.length()] == '*')
		  add_our_call = false;
	      }

	      if (add_our_call) {
		char *s = add_element_to_path(data, Tcall.c_str());
	        if (s) data = s;
	      }
	    }
	  }
	}
	if (lora_tx_enabled) {
          loraSend(txPower, lora_freq, String(data));
          enableOled(); // enable OLED
          writedisplaytext("((KISSTX))","","","","","");
          time_to_refresh = millis() + showRXTime;
	}
out:
        delete TNC2DataFrame;
      }
    }
  #endif

  if (rf95.waitAvailableTimeout(100)) {
    #ifdef T_BEAM_V1_0
      #ifdef ENABLE_LED_SIGNALING
        axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
      #endif
    #endif
    #ifdef BUZZER
      int melody[] = {300, 50, 500, 100};
      buzzer(melody, sizeof(melody)/sizeof(int));
    #endif
      loraReceivedLength = sizeof(lora_RXBUFF);                           // reset max length before receiving!
      if (rf95.recvAPRS(lora_RXBUFF, &loraReceivedLength)) {
        loraReceivedFrameString = "";
        //int rssi = rf95.lastSNR();
        //Serial.println(rssi);
        enableOled(); // enable OLED
        for (int i=0 ; i < loraReceivedLength ; i++) {
          loraReceivedFrameString += (char) lora_RXBUFF[i];
        }
	const char *received_frame = loraReceivedFrameString.c_str();
	// CR adaption: because only for SF12 different CR levels have been defined, we unfortunately cannot deal with SF < 12.
	if (lora_automatic_cr_adaption && lora_speed <= 300L) {
	  // not our own digipeated call?
	  if (! (strncmp(received_frame, Tcall.c_str(), Tcall.length()) == 0 && received_frame[Tcall.length()] == '>')) {
	    lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot++;
	    // was digipeated? -> there was another rf transmission
	    char *p = strchr(received_frame, '>');
	    char *q = strchr(received_frame, ',');
	    char *r = strchr(received_frame, '*');
	    if (p && q && r && q > p && r > q && strchr(r, ':') > r)
	      lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot++;
	    }
	}
	time_last_lora_frame_received = millis();
    #ifdef SHOW_RX_PACKET                                                 // only show RX packets when activitated in config
        writedisplaytext("  ((RX))", "", loraReceivedFrameString, "", "", "");
        #ifdef ENABLE_WIFI
          sendToWebList(loraReceivedFrameString, bg_rf95rssi_to_rssi(rf95.lastRssi()), bg_rf95snr_to_snr(rf95.lastSNR()));
        #endif
        syslog_log(LOG_INFO, String("Received LoRa: '") + loraReceivedFrameString + "', RSSI:" + bg_rf95rssi_to_rssi(rf95.lastRssi()) + ", SNR: " + bg_rf95snr_to_snr(rf95.lastSNR()));
    #endif
        #ifdef KISS_PROTOCOL
	char *s = 0;
	if (lora_add_snr_rssi_to_path)
	  s = add_element_to_path(received_frame, encode_snr_rssi_in_path());
	sendToTNC(s ? String(s) : loraReceivedFrameString);
        #endif

	// Are we configured as lora digi?
	if (lora_digipeating_mode > 0 && lora_tx_enabled) {
	  uint32_t time_lora_TXBUFF_for_digipeating_was_filled_prev = time_lora_TXBUFF_for_digipeating_was_filled;
          handle_lora_frame_for_lora_digipeating(received_frame, encode_snr_rssi_in_path());
	  // new frame in digipeating queue? cross-digi freq enabled and freq set? Send without delay.
          if (*lora_TXBUFF_for_digipeating && lora_cross_digipeating_mode > 0 && lora_freq_cross_digi > 1.0 && lora_freq_cross_digi != lora_freq && time_lora_TXBUFF_for_digipeating_was_filled > time_lora_TXBUFF_for_digipeating_was_filled_prev) {
	    // word NOGATE part of the header? Don't gate it
	    char *q = strstr(lora_TXBUFF_for_digipeating, ",NOGATE");
	    if (!q || q > strchr(lora_TXBUFF_for_digipeating, ':')) {
              ulong lora_speed_prev = lora_speed;
              lora_speed = lora_speed_cross_digi;
              loraSend(txPower, lora_freq_cross_digi, String(lora_TXBUFF_for_digipeating));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
              lora_speed = lora_speed_prev;  // we really need an argument for speed in loraSend()
              writedisplaytext("  ((TX cross-digi))", "", String(lora_TXBUFF_for_digipeating), "", "", "");
#ifdef KISS_PROTOCOL
	      char *s = add_element_to_path(lora_TXBUFF_for_digipeating, "GATE");
	      sendToTNC(s ? String(s) : lora_TXBUFF_for_digipeating);
#endif
	    }
	  }
	}
      }
    #ifdef T_BEAM_V1_0
      #ifdef ENABLE_LED_SIGNALING
        axp.setChgLEDMode(AXP20X_LED_OFF);
      #endif
    #endif
  }

  if (lora_automatic_cr_adaption && lora_speed <= 300L && millis() > (5*60*1000L) && (time_lora_automaic_cr_adoption_rx_measurement_window + 5*60*1000L) < millis()) {
    // recalculate automatic adapted CR
    // transmissions in 5 min window.
    // Approx seconds between frames: if transmission takes with CR4/5 3s and sleeps 3x so long, we see every 25s a transmission in a 5s window.
    // With CR4/6 we have 300 / (3*4) * 300.0/240.0 = 31s; wih CR4/7 we have 35s, and with 41s.
    // Slowly, step by step, incrase CR after frequency becomes quet
    uint32_t t_diff = millis() - time_last_lora_frame_received;
    if (t_diff > (300*1000L / (3*4) * 300.0/180.0) && lora_speed <= 210)
      lora_speed = 180;
    else if (t_diff > (300*1000L / (3*4) * 300.0/210.0) && lora_speed <= 240)
      lora_speed = 210;
    else if (t_diff > (300*1000L / (3*4) * 300.0/240.0) && lora_speed <= 300)
      lora_speed = 240;
    else
      lora_speed = 300;
    lora_automaic_cr_adoption_rf_transmissions_heard_in_timeslot /= 2L;
    time_lora_automaic_cr_adoption_rx_measurement_window = millis() - (5*60*1000L/2L);
  }

  // Send position, if not requested to do not ;) But enter this part if user likes our LA/LON/SPD/CRS to be displayed on his screen ('!gps_allow_sleep_while_kiss' caused 'gps_state false')
  if (!gps_state && (!dont_send_own_position_packets || !lora_tx_enabled))
    goto behind_position_tx;

  LatShown = String(gps.location.lat(),5);
  LongShown = String(gps.location.lng(),5);
  average_speed[point_avg_speed] = gps.speed.kmph();   // calculate smart beaconing
  ++point_avg_speed;
  if (point_avg_speed>4) {
    point_avg_speed=0;
  }

  average_speed_final = (average_speed[0]+average_speed[1]+average_speed[2]+average_speed[3]+average_speed[4])/5;
  average_course[point_avg_course] = gps.course.deg();   // calculate smart beaconing course
  ++point_avg_course;
  
  if (point_avg_course>(ANGLE_AVGS-1)) {
    point_avg_course=0;
    avg_c_y = 0;
    avg_c_x = 0;
    for (int i=0;i<ANGLE_AVGS;i++) {
      avg_c_y += sin(average_course[i]/180*3.1415);
      avg_c_x += cos(average_course[i]/180*3.1415);
    }
    new_course = atan2f(avg_c_y,avg_c_x)*180/3.1415;
    if (new_course < 0) {
        new_course=360+new_course;
      }
    // tooo much false positives
    // Only in 30s interval and if we did not announce turn in last round.
    // Algo from Kenwood SB Docu. Considered values should be int.
    // algo is: (int ) ((int )sb_angle + 10 * turn_slope / (int ) mph). In km/h, we have (int ) ((int ) ab_angle + 16 *turn_slope / (int ) mph)
    if (nextTX > 1 && (millis()-lastTX) > (sb_turn_time*1000L) && average_speed_final >= sb_min_speed) {
        // cave: average_speed_final must not be 0 (division by zero). Becuse sb_min_speed may be configured as zero, check above is not enough
        int int_average_speed_final = (int ) average_speed_final;
        if (int_average_speed_final < 1) int_average_speed_final = 1;
        int sb_turn_threshold=min(120, (int ) ((int ) sb_angle + 16 * sb_turn_slope / int_average_speed_final));
        if ((old_course < sb_turn_threshold) && (new_course > (360-sb_turn_threshold))) {
          if (abs(new_course-old_course-360)>=sb_turn_threshold) {
            nextTX = 1;
          }
        } else {
          if ((old_course > (360-sb_turn_threshold)) && (new_course < sb_turn_threshold)) {
            if (abs(new_course-old_course+360)>=sb_turn_threshold) {
              nextTX = 1;
            }
          } else {
            if (abs(new_course-old_course)>=sb_turn_threshold) {
              nextTX = 1;
            }
          }
      }
    }
    old_course = new_course;
  }

  if ((millis()<sb_max_interval)&&(lastTX == 0)) {
    nextTX = 0;
  }

  // No course change (indicator nextTX==1)? Recomputei nextTX
  if (nextTX > 1 && millis()-lastTX >= sb_min_interval) {
#ifdef SB_ALGO_KENWOOD
    if (average_speed_final < sb_min_speed)
      nextTX = sb_max_interval;
    else if (average_speed_final > sb_max_speed)
      nextTX = sb_min_interval;
    else
      nextTX = ( sb_min_interval * sb_max_speed / average_speed_final);
#else
    // dl9sau: imho, too affine at high speed level
    //nextTX = (sb_max_interval-sb_min_interval)/(sb_max_speed-sb_min_speed)*(sb_max_speed-average_speed_final)+sb_min_interval;
    // next computation could become negative (if we are faster than sb_max_speed). fixes: either work with temporary signed long variable, or test if sb_max_speed < average_speed_final.
    nextTX = ((sb_max_speed > average_speed_final) ? ((sb_max_interval-sb_min_interval)/(sb_max_speed-sb_min_speed)*(sb_max_speed-average_speed_final)+sb_min_interval) : sb_min_interval);
    //if (nextTX < sb_min_interval) {nextTX=sb_min_interval;}   // already assured (  (sb_max_speed <= average_speed_final) -> nextTX=sb_min_interval)
    if (nextTX > sb_max_interval) {nextTX=sb_max_interval;}
 #endif
    // now, nextTX is >= sb_min_interval
  }

  // rate limit to 20s in SF12 CR4/5 aka lora_speed 300; 5s in lora_speed 1200 (SF9 CR4/7). -> 1200/lora_speed*5 seconds == 6000000 / lora_speed ms
  // If special case nextTX <= 1: we already enforced rate-limiting (see course computation)
  if (!dont_send_own_position_packets && lora_tx_enabled && (lastTX+nextTX) < millis() && (nextTX <= 1 || (millis()-lastTX) >= (6000000L / lora_speed ))) {
    if (gps.location.age() < 2000) {
      enableOled(); // enable OLED
      writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),getSatAndBatInfo());
      sendpacket();
      // We just transmitted. We transmitted due to turn? Don't TX again in next round:
      if (nextTX < sb_min_interval) nextTX = sb_min_interval;
    } else {
      if (millis() > time_to_refresh){
        displayInvalidGPS();
      }
    }
  }else{
    if (millis() > time_to_refresh){
      if (gps.location.age() < 2000) {
        writedisplaytext(" "+Tcall,"Time to TX: "+((dont_send_own_position_packets || !lora_tx_enabled) ? "never" : (String(((lastTX+nextTX)-millis())/1000)+"sec")),"LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph())+"  CRS: "+String(gps.course.deg(),1),getSatAndBatInfo());
      } else {
        displayInvalidGPS();
      }
    }
  }

behind_position_tx:

  #if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
    if (nextTelemetryFrame < millis()){
      // Schedule the next telemetry frame
      nextTelemetryFrame = millis() + (tel_interval * 1000);
      sendTelemetryFrame();
    }
  #endif
  #ifdef KISS_PROTOCOL
    #ifdef KISS_DEBUG
      static auto last_debug_send_time = millis();
      if (millis() - last_debug_send_time > 1000*5) {
        last_debug_send_time = millis();
        String debug_message = "";
        #ifdef T_BEAM_V1_0
          debug_message += "Bat V: " + String(axp.getBattVoltage());
          debug_message += ", ";
          debug_message += "Bat IN A: " + String(axp.getBattChargeCurrent());
          debug_message += ", ";
          debug_message += "Bat OUT A: " + String(axp.getBattDischargeCurrent());
          debug_message += ", ";
          debug_message += "USB Plugged: " + String(axp.isVBUSPlug());
          debug_message += ", ";
          debug_message += "USB V: " + String(axp.getVbusVoltage());
          debug_message += ", ";
          debug_message += "USB A: " + String(axp.getVbusCurrent());
          debug_message += ", ";
          debug_message += "Temp C: " + String(axp.getTemp());
        #else
          debug_message += "Bat V: " + String(BattVolts);
        #endif

        Serial.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #ifdef ENABLE_BLUETOOTH
          SerialBT.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #endif
      }
    #endif
  #endif


  // Data for digipeating in queue?
  if (lora_digipeating_mode && *lora_TXBUFF_for_digipeating && lora_tx_enabled) {
    // 5s grace time (plus up to 250ms random) for digipeating. 10s if we are a fill-in digi
    if ((time_lora_TXBUFF_for_digipeating_was_filled + 5*lora_digipeating_mode*1000L + (millis() % 250)) < millis()) {
      if (lora_cross_digipeating_mode < 2 && (time_lora_TXBUFF_for_digipeating_was_filled + 2* 5*lora_digipeating_mode*1000L) > millis()) {
        ulong lora_speed_prev = lora_speed;
        // if SF12: we degipeat in fastest mode CR4/5.
        if (lora_speed < 300)
          lora_speed = 300;
        loraSend(txPower, lora_freq, String(lora_TXBUFF_for_digipeating));  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
        lora_speed = lora_speed_prev;
        writedisplaytext("  ((TX digi))", "", String(lora_TXBUFF_for_digipeating), "", "", "");
#ifdef KISS_PROTOCOL
        sendToTNC(String(lora_TXBUFF_for_digipeating));
#endif
      } // else: too late. skip TX
      *lora_TXBUFF_for_digipeating = 0L;
    }
  }

  vTaskDelay(1);
}
