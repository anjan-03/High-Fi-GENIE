#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP source_udp; 

unsigned int source_port = 1234;
unsigned int intermediate_port = 1234;

const char *ssid = "ESP32";  
const char *password = "12345678";

IPAddress source_ip(192, 168, 4, 1);  
IPAddress intermediate_ip(192, 168, 4, 10);   
IPAddress subnet(255, 255, 255, 0);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // ESP-32 as client
  WiFi.config(source_ip, intermediate_ip, subnet);
  WiFi.begin(ssid,password);
  while (!WiFi.isConnected())
  {
    WiFi.reconnect();
  }
  
  source_udp.begin(source_port);
}

void loop() {
  source_udp.beginPacket(intermediate_ip, intermediate_port);
  source_udp.print("hello from source.");
  source_udp.endPacket();
  delay(1000);

}
