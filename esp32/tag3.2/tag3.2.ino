#include <SPI.h>
#include <DW1000Ranging.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34
#define ONE_WIRE_BUS 14

struct MyLink
{
    uint16_t anchor_addr;
    float range[3];
    float dbm;
    float temperature;
    struct MyLink *next;
};

struct MyLink *init_link();
void add_link(struct MyLink *p, uint16_t addr);
struct MyLink *find_link(struct MyLink *p, uint16_t addr);
void fresh_link(struct MyLink *p, uint16_t addr, float range, float dbm, float temperature);
void print_link(struct MyLink *p);
void delete_link(struct MyLink *p, uint16_t addr);
void make_link_json(struct MyLink *p, String *s);

const char *hostMacAddress = "  C8:F0:9E:BE:FA:B0"; // REPLACE WITH RECEIVER ESP32 MAC ADDRESS

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

struct MyLink *uwb_data;
int index_num = 0;
long runtime = 0;
String all_json = "";

esp_now_peer_info_t peerInfo;
typedef struct {
  char a[256];
  int b;
  float c;
  bool d;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
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

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send callback function
    esp_now_register_send_cb(OnDataSent);

    // Set receiver MAC address
    uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
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
        String json;
        make_link_json(uwb_data, &json);

        // Set values to send
        strcpy(myData.a, json.c_str());
        myData.b = 0;
        myData.c = 0.0;
        myData.d = false;

        // Send message via ESP-NOW
        esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *)&myData, sizeof(myData));

        if (result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }

        runtime = millis();
    }
}

// UWB ranging callback function
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

// Create a link data structure
struct MyLink *init_link()
{
    struct MyLink *p;
    p = (struct MyLink *)malloc(sizeof(struct MyLink));
    p->next = NULL;
    return p;
}

// Add a link to the data structure
void add_link(struct MyLink *p, uint16_t addr)
{
    struct MyLink *temp;
    temp = (struct MyLink *)malloc(sizeof(struct MyLink));
    temp->anchor_addr = addr;
    temp->next = p->next;
    p->next = temp;
}

// Find a link in the data structure
struct MyLink *find_link(struct MyLink *p, uint16_t addr)
{
    while (p->next != NULL)
    {
        p = p->next;
        if (p->anchor_addr == addr)
            return p;
    }
    return NULL;
}

// Update link data
void fresh_link(struct MyLink *p, uint16_t addr, float range, float dbm, float temperature)
{
    struct MyLink *temp;
    temp = find_link(p, addr);
    if (temp == NULL)
    {
        Serial.println("ERROR");
        return;
    }
    temp->range[2] = temp->range[1];
    temp->range[1] = temp->range[0];
    temp->range[0] = range;
    temp->dbm = dbm;
    temp->temperature = temperature;
}

// Print link data
void print_link(struct MyLink *p)
{
    while (p->next != NULL)
    {
        p = p->next;
        Serial.print(p->anchor_addr, HEX);
        Serial.print(" - ");
        Serial.print(p->range[0]);
        Serial.print(" - ");
        Serial.print(p->dbm);
        Serial.print(" - ");
        Serial.println(p->temperature);
    }
}

// Delete a link from the data structure
void delete_link(struct MyLink *p, uint16_t addr)
{
    struct MyLink *temp;
    while (p->next != NULL)
    {
        if (p->next->anchor_addr == addr)
        {
            temp = p->next;
            p->next = temp->next;
            free(temp);
            return;
        }
        p = p->next;
    }
}

// Create a JSON string from link data
// Create a JSON string from link data
void make_link_json(struct MyLink *p, String *s)
{
    *s = "{";
    bool isFirst = true;

    while (p->next != NULL)
    {
        p = p->next;
        if (!isFirst)
            *s += ",";
        else
            isFirst = false;

        String addr = String(p->anchor_addr, HEX);
        *s += "\"" + addr + "\":";
        *s += "{\"range\":" + String(p->range[0]) + ",";
        *s += "\"dbm\":" + String(p->dbm) + ",";
        *s += "\"temperature\":" + String(p->temperature) + "}";
    }

    *s += "}";
}

