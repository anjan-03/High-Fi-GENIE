#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <WiFi.h>
#include "link.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_now.h>


#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34
#define ONE_WIRE_BUS 14

struct MyLink *uwb_data;
  int index_num=0;
  long runtime=0;
  String all_json="";

uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xBE, 0xFA, 0xB0};

typedef struct struct_message { 
struct MyLink *uwb_data ;
int index_num = 0;
long runtime = 0;
String all_json = "";
} struct_message;

// DS18B20 sensor setup
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
   peerInfo.channel=0;
   peerInfo.encrypt = false;

   if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
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
}

void loop()
{
    all_json="";
    DW1000Ranging.loop();
    if ((millis() - runtime) > 1000)
    {
        // Read temperature from the DS18B20 sensor
        sensors.requestTemperatures();
        float temperature = sensors.getTempCByIndex(0);

        // Create JSON string with range and temperature data
        make_link_json(uwb_data, &all_json);
        addTemperatureToJSON(&all_json, temperature);

        // Send data via UDP
        //sendUDP(&all_json);
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&uwb_data, sizeof(uwb_data));
        Serial.println(all_json);
        runtime = millis();
        delay(1000);
    }
}

void newRange()
{
    Serial.print("From: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
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
    fresh_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), DW1000Ranging.getDistantDevice()->getRXPower(), temperature);
    delay(1000);
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




void addTemperatureToJSON(String *json, float temperature)
{
    *json = json->substring(0, json->length() - 1); // Remove the closing brace
    *json += " temperature " + String(temperature);
    *json +="}";
    *json += "\0";
}

