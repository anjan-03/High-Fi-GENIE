#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP intermediate_udp; // Creation of wifi Udp instance

char packet_buffer[255];

unsigned int local_port = 1234;

const char *ssid = "ESP32";  
const char *password = "12345678";
IPAddress intermediate_ip(192, 168, 4, 10);   
IPAddress subnet(255, 255, 255, 0);

void setup() {
  Serial.begin(115200);
  WiFi.softAPConfig(intermediate_ip, intermediate_ip, subnet);
  WiFi.softAP(ssid, password);  // ESP-32 as access point
  intermediate_udp.begin(local_port);
  }

void loop() {
    int packet_size;
    packet_size = intermediate_udp.parsePacket();
    if (packet_size)
    {
        Serial.print("Received packet of size ");
        Serial.println(packet_size);
        Serial.print("From ");
        IPAddress remote = intermediate_udp.remoteIP();
        for (int i = 0; i < 4; i++)
        {
            Serial.print(remote[i], DEC);
            if (i < 3)
            {
                Serial.print(".");
            }
        }
        Serial.print(", port ");
        Serial.println(intermediate_udp.remotePort());

        // read the packet into packetBufffer
        int len = intermediate_udp.read(packet_buffer, 255);
        if (len > 0)
        {
            packet_buffer[len] = 0;
        }
        Serial.println("Contents:");
        Serial.println(packet_buffer);
        
    }
    delay(1000);
    

  
}
