#include <SPI.h>
#include "DW1000Ranging.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define ANCHOR_ADD "81:17:5B:D5:A9:9A:E2:9C" //for anchor A3
uint8_t broadcastAddress2[] = {0xC8, 0xF0, 0x9E, 0xBE, 0xFA, 0xB0}; //b
uint8_t broadcastAddress1[] = {0xC8, 0xF0, 0x9E, 0xBE, 0xFA, 0xFC}; //a
uint8_t broadcastAddress4[] = {0xD4, 0xD4, 0xDA, 0x46, 0x70, 0x58}; //d
uint8_t broadcastAddress5[] = {0xD4, 0xD4, 0xDA, 0x46, 0x72, 0x74}; //e

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

typedef struct struct_message {
  uint8_t disdata[200];
  char path[10];
  int count;
  int length;
  char s_node;
} struct_message;


struct_message receivedData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status);
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&receivedData, data, sizeof(struct_message));
  Serial.print("Bytes Received: ");
  Serial.println(data_len);

  //int length = sizeof(receivedData.disdata) / sizeof(uint8_t);

  // Print the received data
  Serial.print("Received disdata: ");
  for(int i=0; i <receivedData.length;i++){
    Serial.print((char)receivedData.disdata[i]);
  }
  Serial.println();

  for(int j = 0; j<10; j++){
    Serial.print(receivedData.path[j]);
  }
  Serial.println();
  Serial.println(receivedData.count);

  receivedData.count++;
  Serial.println(receivedData.count);
  if ((char)receivedData.path[receivedData.count]=='B'){
      esp_err_t result1 = esp_now_send(broadcastAddress2, (uint8_t *) &receivedData, sizeof(receivedData));
      if (result1==ESP_OK){
            Serial.println("Sent with success to A2");
      }
      else{
          Serial.println("Error sending data");
      }
  }
  else if (receivedData.path[receivedData.count]=='A'){
      esp_err_t result2 = esp_now_send(broadcastAddress1, (uint8_t *) &receivedData, sizeof(receivedData));
      if (result2==ESP_OK){
            Serial.println("Sent with success to A1");
      }
      else{
          Serial.println("Error sending data");
      }
  }
  else if (receivedData.path[receivedData.count]=='D'){
      esp_err_t result3 = esp_now_send(broadcastAddress4, (uint8_t *) &receivedData, sizeof(receivedData));
      if (result3==ESP_OK){
            Serial.println("Sent with success to A4");
      }
      else{
          Serial.println("Error sending data");
      }
  }
  else if (receivedData.path[receivedData.count]=='E'){
      esp_err_t result4 = esp_now_send(broadcastAddress5, (uint8_t *) &receivedData, sizeof(receivedData));
      if (result4==ESP_OK){
            Serial.println("Sent with success to A5");
      }
      else{
          Serial.println("Error sending data");
      }
  }
  else{
    Serial.println("Invalid conditions!");
    Serial.print(receivedData.path[0]);
  }
}


// Connection pins
const uint8_t PIN_RST = 27; // Reset pin
const uint8_t PIN_IRQ = 34; // IRQ pin
const uint8_t PIN_SS = 4;   // SPI select pin

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);


  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  delay(1000);

  // Initialize DW1000 module
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachBlinkDevice(newBlink);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // Start the module as an anchor
  DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {
  Serial.print("From: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\tRange: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  Serial.print(" m");
  Serial.print("\tRX power: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  Serial.println(" dBm");
}

void newBlink(DW1000Device *device) {
  Serial.print("Blink; 1 device added! -> ");
  Serial.print("Short: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}


