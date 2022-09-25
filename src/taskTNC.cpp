#include "taskTNC.h"
#include <esp_task_wdt.h>


extern boolean usb_serial_data_type;

#ifdef ENABLE_BLUETOOTH
  BluetoothSerial SerialBT;
#endif

QueueHandle_t tncReceivedQueue = nullptr;
#ifdef ENABLE_WIFI
  #define MAX_WIFI_CLIENTS 6
  WiFiClient * clients[MAX_WIFI_CLIENTS];
#endif
#ifdef ENABLE_WIFI
  #define IN_TNC_BUFFERS (2+MAX_WIFI_CLIENTS)
#else
  #define IN_TNC_BUFFERS 2
#endif

String inTNCDataBuffers[IN_TNC_BUFFERS];

QueueHandle_t tncToSendQueue = nullptr;


/**
 * Handle incoming TNC KISS data character
 * @param character
 */
void handleKISSData(char character, int bufferIndex) {
  String *inTNCData = &inTNCDataBuffers[bufferIndex];
  if (inTNCData->length() == 0 && character != (char) FEND){
    // kiss frame begins with C0
    return;
  }
  inTNCData->concat(character);
  if (character == (char) FEND && inTNCData->length() > 3) {
    bool isDataFrame = false;
    const String &TNC2DataFrame = decode_kiss(*inTNCData, isDataFrame);

    if (isDataFrame) {
      #ifdef LOCAL_KISS_ECHO
        if (!usb_serial_data_type) {
          Serial.print(inTNCData);
        }
        #ifdef ENABLE_BLUETOOTH
        if (SerialBT.hasClient()) {
          SerialBT.print(inTNCData);
        }
        #endif
          #ifdef ENABLE_WIFI
        iterateWifiClients([](WiFiClient *client, const String *data){
          if (client->connected()){
            client->print(*data);
            client->flush();
          }
        }, &inTNCData, clients, MAX_WIFI_CLIENTS);
          #endif
      #endif
      auto *buffer = new String();
      buffer->concat(TNC2DataFrame);
      if (xQueueSend(tncToSendQueue, &buffer, (1000 / portTICK_PERIOD_MS)) != pdPASS) {
        delete buffer;
      }
    }
    inTNCData->clear();
  }
  if (inTNCData->length() > 255){
    // just in case of garbage input reset data
    inTNCData->clear();
  }
}


[[noreturn]] void taskTNC(void *parameter) {
  tncToSendQueue = xQueueCreate(4,sizeof(String *));
  tncReceivedQueue = xQueueCreate(4,sizeof(String *));
  String *loraReceivedFrameString = nullptr;

  esp_task_wdt_init(120, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  while (true) {
    esp_task_wdt_reset();
    if (!usb_serial_data_type) {
      while (Serial.available() > 0) {
        char character = Serial.read();
        handleKISSData(character, 0);
      }
    }
    #ifdef ENABLE_BLUETOOTH
      if (SerialBT.hasClient()) {
        while (SerialBT.available() > 0) {
          char character = SerialBT.read();
          handleKISSData(character, 1);
        }
      }
    #endif
    #if defined(ENABLE_WIFI) && defined(KISS_PROTOCOL)
      check_for_new_clients(&tncServer, clients, MAX_WIFI_CLIENTS);

      iterateWifiClients([](WiFiClient * client, int clientIdx, const String * unused){
        while (client->available() > 0) {
          char character = client->read();
          handleKISSData(character, 2+clientIdx);
        }
      }, nullptr, clients, MAX_WIFI_CLIENTS);

    #endif
    if (xQueueReceive(tncReceivedQueue, &loraReceivedFrameString, (1 / portTICK_PERIOD_MS)) == pdPASS) {
      const String &kissEncoded = encode_kiss(*loraReceivedFrameString);
      if (!usb_serial_data_type)
        Serial.print(kissEncoded);
      #ifdef ENABLE_BLUETOOTH
        if (SerialBT.hasClient()){
          SerialBT.print(kissEncoded);
        }
      #endif
      #ifdef ENABLE_WIFI
        iterateWifiClients([](WiFiClient *client, int clientIdx, const String *data){
          if (client->connected()){
            client->print(*data);
            client->flush();
          }
        }, &kissEncoded, clients, MAX_WIFI_CLIENTS);
      #endif

      delete loraReceivedFrameString;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

