#include <taskGPS.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <taskWebServer.h>
#include <esp_task_wdt.h>


SFE_UBLOX_GPS myGPS;

extern uint8_t usb_serial_data_type;
extern volatile boolean gps_task_enabled;

#ifdef ENABLE_WIFI
  #include "wifi_clients.h"
  #define MAX_GPS_WIFI_CLIENTS 6
  WiFiClient * gps_clients[MAX_GPS_WIFI_CLIENTS];
#endif

// Pins for GPS
#if defined(T_BEAM_V1_0) || defined(T_BEAM_V1_2)
  static const int RXPin = 12, TXPin = 34;
#else /* i.e. T_BEAM_V0_7, or lora32-device-with-self-attached-GPS */
  static const int RXPin = 15, TXPin = 12;
#endif
#ifndef LORA32_21
static const uint32_t GPSBaud = 9600; //GPS
#else /* one user played with self-attached GPS on his LORA32 device. TODO: gps speed choosable in Web-Interface */
static const uint32_t GPSBaud = 57600; //GPS
#endif
HardwareSerial gpsSerial(1);        // TTGO has HW serial
TinyGPSPlus gps;             // The TinyGPS++ object
bool gpsInitialized = false;

[[noreturn]] void taskGPS(void *parameter) {

reinitialize:
  if (!gpsInitialized){

    gpsSerial.begin(GPSBaud, SERIAL_8N1, TXPin, RXPin);        //Startup HW serial for GPS

    // set GPS parameters on restart
    // Thanks Peter (https://github.com/peterus)
    // https://github.com/lora-aprs/TTGO-T-Beam_GPS-reset
    if(myGPS.begin(gpsSerial)){
          myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
          //myGPS.saveConfiguration(); //Save the current settings to flash and BBR
          myGPS.enableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
          myGPS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
          myGPS.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
          myGPS.enableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
          myGPS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
          myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
          //myGPS.saveConfiguration(); //Save the current settings to flash and BBR    
          delay(1000);
    }
  }
  gps_task_enabled = true;

  // esp_task_wdt_init() has already been done in main task during setup()
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  String gpsDataBuffer = "              ";
  for (;;) {
    if (!gps_task_enabled) {
      //gpsSerial.end();  // No, raises exception
      esp_task_wdt_delete(NULL);
      gpsInitialized = false;
      vTaskSuspend(NULL);
      goto reinitialize;
    }
    esp_task_wdt_reset();
    #ifdef ENABLE_WIFI
    check_for_new_clients(&gpsServer, gps_clients, MAX_GPS_WIFI_CLIENTS);
    #endif
    while (gpsSerial.available() > 0) {
      char gpsChar = (char)gpsSerial.read();
      gps.encode(gpsChar);
        if (gpsChar == '$') {
          gpsDataBuffer = String(gpsChar);
        } else {
          gpsDataBuffer += String(gpsChar);

          if (gpsChar == '\n') {
            if ((usb_serial_data_type & 4))
              Serial.println(gpsDataBuffer);
      #ifdef ENABLE_WIFI
            iterateWifiClients([](WiFiClient *client, int clientIdx, const String *data){
              if (client->connected()){
                client->print(*data);
                client->flush();
              }
            }, &gpsDataBuffer, gps_clients, MAX_GPS_WIFI_CLIENTS);
      #endif
            gpsDataBuffer = "";
          }
        }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
