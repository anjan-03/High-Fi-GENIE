#include <SPI.h>
#include "DW1000Ranging.h"
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define ANCHOR_ADD "83:17:5B:D5:A9:9A:E2:9C" //for anchor A1

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

typedef struct struct_message {
  char a[200];
} struct_message;

struct_message receivedData;

esp_now_peer_info_t peerInfo;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&receivedData, data, sizeof(struct_message));
  Serial.print("Bytes Received: ");
  Serial.println(data_len);
  Serial.print("Received data: ");
  
  for (int i = 0; i < data_len; i++) {
    Serial.print((char)data[i]);
  }
  
  Serial.println();
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

  // Set receiver MAC address
  uint8_t receiverMacAddress[] = {0xC8, 0xF0, 0x9E, 0xBE, 0xFB, 0x20};
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);

  // Add peer
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
