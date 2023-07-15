#include <SPI.h>
#include <DW1000RangingClass.h>

#define ANCHOR_ADD "12:34:56:78:9A:BC"  // Replace with the MAC address of the anchor device

// Receiver MAC address (Anchor MAC address)
uint8_t receiverMac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};

void setup() {
  Serial.begin(115200);

  // Initialize the DW1000 module
  DW1000Ranging.init(DW1000RANGING_ANCHOR);
  
  // Start the ranging process as a tag
  DW1000Ranging.attachEventHandler(onRangingEvent);
  DW1000Ranging.start(ANCHOR_ADD);
}

void loop() {
  // Perform ranging request to the anchor
  DW1000Ranging.newTransmit(receiverMac);
  DW1000Ranging.startTransmit();

  // Wait for the ranging result
  delay(1000);

  // Get the range result from the DW1000 module
  DW1000Ranging.getReceiveTimestamp();
  DW1000Ranging.getTransmitTimestamp();

  // Process the ranging result if available
  if (DW1000Ranging.rangingComplete()) {
    float range = DW1000Ranging.getRange();
    Serial.print("Anchor Range: ");
    Serial.print(range);
    Serial.println(" m");

    // Additional processing or actions based on the range result can be implemented here

    // Reset the ranging device for the next ranging request
    DW1000Ranging.reset();
  }

  delay(5000); // Wait for 5 seconds before performing the next ranging request
}

void onRangingEvent() {
  // Handle ranging events if necessary
}
