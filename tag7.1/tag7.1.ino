#include <SPI.h>
#include <DW1000Ranging.h>
#include <WiFi.h>
#include "link.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_now.h>
#include <math.h>
#include <string.h>
#include <float.h>
#include <WiFiUdp.h>


#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34
#define ONE_WIRE_BUS 14
const char* ssid = "STUDENTS";
const char* password = "students123";
const IPAddress piIP(172, 16, 19, 11);
float d1 = 100; // dist between tag and A1
float d2 = 100; //distance between tag and A2
float d3 = 100; //distance between tag and A3
float d4 = 100; //distance between tag and A4

WiFiUDP udp;

float x; //coordinates for tag
float y;

int tempadd;

int fcount=0;

int a1 = 6019;  // address of A1 1783
int a2 = 6018;  // address of A2 1782
int a3 = 6017;  // address of A3 1781
int a4 = 6016;  // address of A4 1780

float smallest1 = FLT_MAX;
float smallest2 = FLT_MAX;

float ax=2, ay=0, bx=5, by=5, cx=0, cy=5, dx=3, dy=1;


char nodes[] = {'A', 'B', 'C', 'D', 'E'};
float distances[5][5] = {
    {0, 5.83, 5.38, 1.41, 2},
    {5.83, 0, 5, 4.47, 7.07},
    {5.38, 5, 0, 5, 5},
    {1.41, 4.47, 5, 0, 3.16},
    {2, 7.07, 5, 3.16, 0}

};

uint8_t broadcastAddress1[] = {0xC8, 0xF0, 0x9E, 0xBE, 0xFA, 0xFC};   // to A1
uint8_t broadcastAddress2[] = {0xC8, 0xF0, 0x9E, 0xBE, 0xFA, 0xB0};   // to A2
uint8_t broadcastAddress3[] = {0x54, 0x43, 0xB2, 0x7F, 0x56, 0xBC};   // to A3
uint8_t broadcastAddress4[] = {0xD4, 0xD4, 0xDA, 0x46, 0x70, 0x58};   // to A4
uint8_t broadcastAddress5[] = {0x40, 0x91, 0x51, 0xFB, 0xDF, 0x04};   // to S1

struct MyLink *uwb_data;
int index_num = 0;
long runtime = 0;
String all_json = "";

typedef struct algdata {
  uint8_t disdata[200];
  char path[10];
  int count;
  int length;
  char s_node;
} algdata;

algdata data;

// DS18B20 sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
esp_now_peer_info_t peerInfo;

char first_node;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status != ESP_NOW_SEND_SUCCESS) {
      fcount++;
  }
}

void setup()
{
    Serial.begin(115200);

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
   
   if (esp_now_init() != ESP_OK){
     Serial.println("Error Initializing ESP-NOW");
     return;
   }

  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 1");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 2");
    return;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 3");
    return;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 4");
    return;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 5");
    return;
  }


  delay(1000);

    // Initialize DW1000 module
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    // Start the module as a tag
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

    // Initialize the link data structure
    uwb_data = init_link();

    // Initialize the DS18B20 sensor
    sensors.begin();

    /*data.path[0]=6019;
    data.path[1]=6018;
    data.path[2]=6017;*/
    data.count=0;

}

void loop()
{
   
    DW1000Ranging.loop();
    if ((millis() - runtime) > 1000)
    {
        // Read temperature from the DS18B20 sensor
        sensors.requestTemperatures();
        float temperature = sensors.getTempCByIndex(0);

        // Create JSON string with range and temperature data
        make_link_json(uwb_data, &all_json);
        addTemperatureToJSON(&all_json, temperature);

        // Convert all_json to a byte array
        
        /*uint8_t all_json_bytes[all_json.length()];
        all_json.getBytes(all_json_bytes, all_json.length());*/
        uint8_t all_json_bytes[all_json.length() + 1];  // Add 1 for null terminator
        strcpy((char*)all_json_bytes, all_json.c_str());  // Convert string to byte array
        int length_of_arr = sizeof(all_json_bytes);
        memcpy(data.disdata, all_json_bytes, sizeof(all_json_bytes));
        data.length=length_of_arr;
        Serial.print("Length of all_json_bytes is : ");
        Serial.println(data.length);
        Serial.print("The starting node is:");
        Serial.println(first_node);

      if (d1<d2 && d1<d3 && d1<d4){ //to A1
          first_node='A';
          data.s_node=first_node;
          esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *) &data, sizeof(data));
        
          if (result1==ESP_OK){
            Serial.println("Sent with success to A1");
          }
          else{
          Serial.println("Error sending data");
          fcount++;
          }
      }
      else if (d2<d1 && d2<d3 && d2<d4){ // to A2
          first_node='B';
          data.s_node=first_node;
          esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *) &data, sizeof(data));
        
          if (result2==ESP_OK){
            Serial.println("Sent with success to A2");
          }
          else{
            Serial.println("Error sending data");  // in case if failure send data to A1, potential for recursive function?? try until it sends or send for fixed attempts
            fcount++;
          }
      }

      else if (d3<d1 && d3<d2 && d3<d4){ // to A3
          first_node='C';
          data.s_node=first_node;
          esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *) &data, sizeof(data));
        
          if (result3==ESP_OK){
            Serial.println("Sent with success to A3");
          }
          else{
            Serial.println("Error sending data");  // in case if failure send data to A1, potential for recursive function?? try until it sends or send for fixed attempts
            fcount++;
            Serial.println(result3);
          }
      }
      else if (d4<d1 && d4<d2 && d4<d3){ // to A4
          first_node='D';
          data.s_node=first_node;
          esp_err_t result4 = esp_now_send(broadcastAddress4, (uint8_t *) &data, sizeof(data));
        
          if (result4==ESP_OK){
            Serial.println("Sent with success to A4");
          }
          else{
            Serial.println("Error sending data");  // in case if failure send data to A1, potential for recursive function?? try until it sends or send for fixed attempts
            fcount++;
            Serial.println(result4);
          }
      }
        Serial.println(all_json);
        /*for(int i=0; i<sizeof(all_json_bytes); i++){
          Serial.println(*ptr+i);
        }*/
        delay(1000);
        runtime = millis();
    }
  if (fcount == 5) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    sendUDPMessage("HUGE FIRE! ");
    fcount = 2;
  }
}

void newRange()
{
    Serial.print("From: ");
    tempadd=(DW1000Ranging.getDistantDevice()->getShortAddress());
    Serial.print(tempadd);
    Serial.print("\tRange: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\tRX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
    float temperature = sensors.getTempCByIndex(0);
    Serial.println("\tTemperature : ");
    Serial.print(temperature);
    Serial.println(" C");
    fresh_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), temperature);
    if (tempadd==a1){
      d1=DW1000Ranging.getDistantDevice()->getRange();
      //Serial.println(d1);
    }
    else if (tempadd==a2){
      d2=DW1000Ranging.getDistantDevice()->getRange();
      //Serial.println(d2);
    }
    else if (tempadd==a3){
      d3=DW1000Ranging.getDistantDevice()->getRange();
    }
    else if (tempadd==a4){
      d4=DW1000Ranging.getDistantDevice()->getRange();
    }
    if(d4 > d1 && d4 > d2 && d4 > d3){
      x=findPointx(ax,ay,d1,bx,by,d2,cx,cy,d3);
      y=findPointy(ax,ay,d1,bx,by,d2,cx,cy,d3);
    }
    else if(d3 > d1 && d3 > d2 && d3 > d4){
      x=findPointx(ax,ay,d1,bx,by,d2,dx,dy,d4);
      y=findPointy(ax,ay,d1,bx,by,d2,dx,dy,d4);
    }
    else if(d2 > d1 && d2 > d3 && d2 > d4){
      x=findPointx(ax,ay,d1,dx,dy,d4,cx,cy,d3);
      y=findPointy(ax,ay,d1,dx,dy,d4,cx,cy,d3);
    }
    else if(d1 > d3 && d1 > d2 && d1 > d4){
      x=findPointx(dx,dy,d4,bx,by,d2,cx,cy,d3);
      y=findPointy(dx,dy,d4,bx,by,d2,cx,cy,d3);
    }
    func(x,y);
    Serial.print("X coordinate: ");
    Serial.println(x);
    Serial.print("Y coordinate: ");
    Serial.println(y);
}

void newDevice(DW1000Device *device)
{
    Serial.print("Ranging init; 1 device added! -> ");
    Serial.print("Short: ");
    Serial.println(device->getShortAddress(), HEX);

    add_link(uwb_data, device->getShortAddress());
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("Delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);

    delete_link(uwb_data, device->getShortAddress());
}

float findPointx(float x1, float y1, float d1, float x2, float y2, float d2, float x3, float y3, float d3) {

    
    float A = 2 * (x2 - x1);
    float B = 2 * (y2 - y1);
    float C = pow(d1, 2) - pow(d2, 2) - pow(x1, 2) + pow(x2, 2) - pow(y1, 2) + pow(y2, 2);
    
    float D = 2 * (x3 - x2);
    float E = 2 * (y3 - y2);
    float F = pow(d2, 2) - pow(d3, 2) - pow(x2, 2) + pow(x3, 2) - pow(y2, 2) + pow(y3, 2);
    
    x = (C*E - F*B) / (E*A - B*D);
    return x;
    //printf("The point is (%lf, %lf)\n", x, y);
}

float findPointy(float x1, float y1, float d1, float x2, float y2, float d2, float x3, float y3, float d3) {

    
    float A = 2 * (x2 - x1);
    float B = 2 * (y2 - y1);
    float C = pow(d1, 2) - pow(d2, 2) - pow(x1, 2) + pow(x2, 2) - pow(y1, 2) + pow(y2, 2);
    
    float D = 2 * (x3 - x2);
    float E = 2 * (y3 - y2);
    float F = pow(d2, 2) - pow(d3, 2) - pow(x2, 2) + pow(x3, 2) - pow(y2, 2) + pow(y3, 2);
    
    y = (C*D - A*F) / (B*D - A*E);
    return y;
    //printf("The point is (%lf, %lf)\n", x, y);
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
    float ad = sqrt(pow((x - 2), 2) + pow((y - 0), 2));
    //ad = round(ad * 10) / 10;
    float bd = sqrt(pow((x - 5), 2) + pow((y - 5), 2));
    //bd = round(bd * 10) / 10;
    float cd = sqrt(pow((x - 0), 2) + pow((y - 5), 2));
    //cd = round(cd * 10) / 10;
    float dd = sqrt(pow((x - 3), 2) + pow((y - 1), 2));
    //dd = round(dd * 10) / 10;
Serial.print(ad);
Serial.print(bd);
Serial.print(cd);
Serial.print(dd);
    char start_node;
    if (ad <= bd && ad <= cd && ad <= dd)
        start_node = 'A';
    else if (bd <= ad && bd <= cd && bd <= dd)
        start_node = 'B';
    else if (cd <= ad && cd <= bd && cd <= dd)
        start_node = 'C';
    else
        start_node = 'D';

    const char* start_nodeptr = &start_node;

    float visited[5];
    char prevNode[5];
    dijkstra(start_node, nodes, distances, visited, prevNode);
    int counter = 0;
    Serial.print("Shortest distance to E: ");
    Serial.println(visited['E' - 'A']);
    Serial.print("Path to E: ");
    counter=printPath(start_node, 'E', prevNode, counter);
    Serial.print("\n");
    //Serial.print("this is the starting node: ");
    //Serial.println(start_node);
    //Serial.print(ad); Serial.print(bd); Serial.print(cd); Serial.print(dd);
    //data.s_node=start_node;
}

int printPath(char start, char end, char *prevNode, int counter) {
    if (start == end) {
        Serial.print(start);
        data.path[0]=start;
        return 0;
    }
    counter=printPath(start, prevNode[end - 'A'], prevNode, counter);
    data.path[counter+1]=end;
    Serial.print(end);
    counter++;
    return counter;
}


void addTemperatureToJSON(String *json, float temperature)
{
    *json = json->substring(0, json->length() - 1); // Remove the closing brace
    *json += ",\"temperature\": " + String(temperature);
    *json += "}";
}

void sendUDPMessage(const char* message) {
  udp.beginPacket(piIP, 1234); 
  udp.print(message);
  udp.endPacket();
}
