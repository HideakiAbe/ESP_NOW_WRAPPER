# ESP_NOW


## Notice


## History



## Dependencies

#include <esp_now.h>

### PlatformIO


### Arduino IDE



## Usage

The library is very easy to use, basically instantiate an object, connect to the Wifi, add one or more virtual devices and bind the callback to get the messages. An schematic example pair could be:


-----Server side---------
#include "ESPNow.h"
#include <WiFi.h>




void setup() {
  Serial.begin(115200);
}

void loop() {
  static ESPNow mySTREAM("Slave",ESP_NOW_MASTER);
  mySTREAM.write("hello world\n");
  delay(6000);
}
//end of server side



//-----Slave side------------
#include "ESPNOW.h"
#include <WiFi.h>

void setup() {
    Serial.begin(115200);
}

void loop() {
    static ESPNow mySTREAM("Slave", ESP_NOW_SLAVE);
    Serial.print((char)mySTREAM.read());
}

//end of slave side
