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
  char path[10];
  int count=0;
} struct_message;

struct_message receivedData;


esp_now_peer_info_t peerInfo;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  memcpy(&receivedData, data, sizeof(struct_message));
  Serial.print("Bytes Received: ");
  Serial.println(data_len);
  Serial.print("Received data: ");
  
  int count_close=0;

  for (int i = 0; i < data_len; i++) {
    if(toascii((char)data[i])==32 || toascii((char)data[i])== 34 || toascii((char)data[i])==44 || (toascii((char)data[i])>=48 && toascii((char)data[i])<=58) || (toascii((char)data[i])>=65 && toascii((char)data[i])<=90) || (toascii((char)data[i])>=97 && toascii((char)data[i])<=123) || toascii((char)data[i])==125 || toascii((char)data[i])==45 || toascii((char)data[i])==46){
      if(toascii((char)data[i])==125){
        count_close++;
        //Serial.println(count_close);
      }
      if(count_close>1){
          break;
      }
      Serial.print((char)data[i]);
    }
  }
  Serial.print("}");
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

void dijkstra(char current, char nodes[], float distances[5][5], float *visited, char *prevNode) {
    float unvisited[5];
    float currentDistance = 0;
    int i, j;

    for (i = 0; i < 5; i++) {
        unvisited[i] = INFINITY;
        visited[i] = INFINITY;
        prevNode[i] = '\0';  // Initialize previous node as null character
    }

    unvisited[current - 'A'] = 0;

    while (1) {
        for (j = 0; j < 5; j++) {
            char neighbour = nodes[j];
            float distance = distances[current - 'A'][j];    
            if (distance == 0 || visited[j] <= currentDistance + distance)
                continue;

            float newDistance = currentDistance + distance;
            int neighbourIndex = neighbour - 'A';
            if (unvisited[neighbourIndex] == INFINITY || unvisited[neighbourIndex] > newDistance) {
                unvisited[neighbourIndex] = newDistance;
                prevNode[neighbourIndex] = current;  // Set previous node for the neighbor
            }
        }

        visited[current - 'A'] = currentDistance;
        unvisited[current - 'A'] = INFINITY;
        
        int done = 1;
        float minDistance = INFINITY;
        char nextNode;
        for (i = 0; i < 5; i++) {
            if (unvisited[i] < INFINITY) {
                done = 0;
                if (unvisited[i] < minDistance) {
                    minDistance = unvisited[i];
                    nextNode = nodes[i];
                }
            }
        }

        if (done)
            break;
        current = nextNode;
        currentDistance = minDistance;
    }
}

void func(float x, float y) {
    float ad = sqrt(pow((x - 100), 2) + pow((y - 100), 2));
    ad = round(ad * 10) / 10;
    float bd = sqrt(pow((x - 500), 2) + pow((y - 100), 2));
    bd = round(bd * 10) / 10;
    float cd = sqrt(pow((x - 500), 2) + pow((y - 500), 2));
    cd = round(cd * 10) / 10;
    float dd = sqrt(pow((x - 100), 2) + pow((y - 500), 2));
    dd = round(dd * 10) / 10;

    char start_node;
    if (ad <= bd && ad <= cd && ad <= dd)
        start_node = 'A';
    else if (bd <= ad && bd <= cd && bd <= dd)
        start_node = 'B';
    else if (cd <= ad && cd <= bd && cd <= dd)
        start_node = 'C';
    else 
        start_node = 'D';

    float visited[5];
    char prevNode[5];
    dijkstra(start_node, nodes, distances, visited, prevNode);

    Serial.print("Shortest distance to E: ");
    Serial.println(visited['E' - 'A']);
    Serial.print("Path to E: ");
    printPath(start_node, 'E', prevNode);
    Serial.print("\n");
}

void printPath(char start, char end, char *prevNode) {
    if (start == end) {
        Serial.print(start);
        return;
    }
    printPath(start, prevNode[end - 'A'], prevNode);
    Serial.print(end);
    return;
}
