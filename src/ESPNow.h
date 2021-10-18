#include <esp_now.h>
#include <WiFi.h>
//#include  "debugAbe.h"

#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 0
const char *passWord = "Slave_1_Password";

typedef enum {
  ESP_NOW_NOT_DEFINED = -1,       /**< Send ESPNOW data successfully */
  ESP_NOW_MASTER = 1,
  ESP_NOW_SLAVE = 0, /**< Send ESPNOW data fail */
} esp_now_maserORSlave_t;

//void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
//void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
//void InitESPNow(void);

portMUX_TYPE espNowMux = portMUX_INITIALIZER_UNLOCKED;


static    QueueHandle_t xQueue;

struct espNowStruct {
  uint8_t mac_addr[6];
  uint8_t data[250];
  int data_len;
};
class ESPNow: public Print, public Stream {
  public:
    ESPNow(String slaveSSID, esp_now_maserORSlave_t Master_TRUE_Slave_FALSE);
    virtual size_t write(uint8_t);
    virtual size_t write(const char *str);
    virtual size_t write(const uint8_t *buffer, size_t size);
    virtual size_t read(const uint8_t *buffer, size_t size);
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();
    String senderMac();

  private:

    esp_now_maserORSlave_t masterOrSlave = ESP_NOW_NOT_DEFINED;
    String _controlerSSID;
    void InitESPNow(void);
    esp_now_peer_info_t slave;
    bool slaveFound = 0;
    void ScanForSlave(String slaveSSID);
    void deletePeer();
    boolean manageSlave();

    void configDeviceAP(String mySSID);
    static void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
    static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    String newUniqSSID(String originalSSID);
    struct espNowStruct internalBuff;
    int  readingTop=0;

};
void ESPNow::OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  struct espNowStruct newData;
  memcpy(newData.data, data, data_len);
  newData.data_len = data_len;
  for (int i = 0; i < 6; i++) {
    newData.mac_addr[i] = mac_addr[i];
  }
  BaseType_t result = xQueueSendToBack(xQueue, &newData, portMAX_DELAY);
  if (result != pdPASS) {
    //DEBUGOUT("ESP_NOW_ Queue occupied")
    //DEBUGSTOP
  }
}
int ESPNow::available() {
  struct espNowStruct waiting;
  UBaseType_t uxNumberOfItems = uxQueueMessagesWaiting( xQueue );
  if (uxNumberOfItems >= 1) {
    xQueuePeek(xQueue, &waiting, portMAX_DELAY);
    //DEBUGOUT(readingTop)
    return waiting.data_len;
  }
  return 0;
}
int ESPNow::peek() {
  int ret = -1;
  if (internalBuff.data_len > readingTop ) {
    return internalBuff.data[readingTop];
  } else {
    if (uxQueueMessagesWaiting( xQueue )) {
      xQueueReceive(xQueue, &internalBuff, portMAX_DELAY);
      readingTop = 0;
      if (internalBuff.data_len > readingTop ) {
        return internalBuff.data[readingTop];
      }
    }
  }
  return ret;
}
String ESPNow::senderMac() {
  struct espNowStruct waiting;
  String mac = "";
  //DEBUGOUT(uxQueueMessagesWaiting( xQueue ))
  if (internalBuff.data_len > readingTop) {
    for (int i = 0; i < 6; i++) {
      //DEBUGOUTH(waiting.mac_addr[i])
      //DEBUGOUT(String(waiting.mac_addr[i],HEX))
      mac += String(internalBuff.mac_addr[i], HEX) ;
      if (i < 5) {
        mac += ":";
      }
    }
    return mac;
  } else  if (uxQueueMessagesWaiting( xQueue )) {
    xQueuePeek(xQueue, &waiting, portMAX_DELAY);
    if (waiting.mac_addr[0]) {
      for (int i = 0; i < 6; i++) {
        //DEBUGOUTH(waiting.mac_addr[i])
        //DEBUGOUT(String(waiting.mac_addr[i],HEX))
        mac += String(waiting.mac_addr[i], HEX) ;
        if (i < 5) {
          mac += ":";
        }
      }
    }
  }
  return mac;
}
void ESPNow::flush() {
  while (uxQueueMessagesWaiting( xQueue )) {
    xQueueReceive(xQueue, &internalBuff, portMAX_DELAY);
  }
  internalBuff.data_len = 0;
  readingTop = 0;
}
int ESPNow::read() {
  int ret = -1;
  if (internalBuff.data_len > readingTop ) {
    return internalBuff.data[readingTop++];
  } else {
    if (uxQueueMessagesWaiting( xQueue )) {
      xQueueReceive(xQueue, &internalBuff, portMAX_DELAY);
      readingTop = 0;
      if (internalBuff.data_len > readingTop ) {
        return internalBuff.data[readingTop++];
      }
    }
  }
  return ret;
}

size_t ESPNow::read(const uint8_t *buffer, size_t size) {
  uint8_t *p = ( uint8_t *)buffer;
  int reqSize=int(size);
  size_t actual = 0;
  while (reqSize) {
    while (internalBuff.data_len > readingTop ) {
      *p++ = internalBuff.data[readingTop++];
      reqSize--;
      actual++;
      //DEBUGOUT(reqSize)
      if (reqSize <= 0) break;
    }
    if (reqSize <= 0) break;
    if (uxQueueMessagesWaiting( xQueue )) {
      xQueueReceive(xQueue, &internalBuff, portMAX_DELAY);
      readingTop = 0;
    }else{
      break;
    }
    //DEBUGOUT(internalBuff.data_len)
    //DEBUGOUT(readingTop)
    
  }
  return actual;
}
ESPNow::ESPNow(String slaveSSID, esp_now_maserORSlave_t MasterORSlave) {
  masterOrSlave = MasterORSlave;
  _controlerSSID = slaveSSID;
  readingTop=0;
  if (masterOrSlave == ESP_NOW_MASTER) {
    WiFi.mode(WIFI_STA);
  } else if ( masterOrSlave == ESP_NOW_SLAVE) {
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    configDeviceAP(_controlerSSID);
  } else {
    //DEBUGOUT("ESP_NOW_NOT_DEFINED cannot use")
    //DEBUGSTOP
  }
  InitESPNow();
  esp_now_register_send_cb(ESPNow::OnDataSent);
  esp_now_register_recv_cb(ESPNow::OnDataRecv);
  if (MasterORSlave == true) {
    ScanForSlave(_controlerSSID);
    manageSlave();
  }
  xQueue = xQueueCreate(4, sizeof(espNowStruct));
  if (xQueue == 0) {
    //DEBUGOUT("ESP_NOW_can not create Queue")
    //DEBUGSTOP
  }
}
void ESPNow::InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    //Serial.println("ESPNow Init Success");
  }
  else {
    //DEBUGOUT("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}
void ESPNow::ScanForSlave(String slaveSSID) {
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  memset(&slave, 0, sizeof(slave));
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    if (PRINTSCANRESULTS) {
      Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    }
    for (int i = scanResults - 1; i >= 0 ; i--) {
      //LINE
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);
      if (1) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf(slaveSSID) == 0) {
        // SSID of interest
        //Serial.println("Found a Slave.");
        //Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        Serial.printf(" % 02x: % 02x: % 02x: % 02x: % 02x: % 02x",
                      slave.peer_addr[0], slave.peer_addr[1], slave.peer_addr[2],
                      slave.peer_addr[3], slave.peer_addr[4], slave.peer_addr[5]);

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }
  if (slaveFound) {
    //DEBUGOUT("Slave Found, processing..");
  } else {
    //DEBUGOUT("Slave Not Found, trying again.");
  }
  // clean up ram
  WiFi.scanDelete();
}

size_t ESPNow::write(const char *str) {
  esp_err_t result = !ESP_OK;
  size_t strlength = 0;
  char *p = (char *) str;
  while (*p) {
    p++;
    strlength++;
    if (strlength >= ESP_NOW_MAX_DATA_LEN) {
      break;
    }
  }
  return write((const uint8_t *)str,  strlength);
}
size_t ESPNow::write(const uint8_t data) {
  esp_err_t result = !ESP_OK;
  size_t actual = 0;
  if (manageSlave()) {
    if (slaveFound) {
      const uint8_t *peer_addr = slave.peer_addr;
      Serial.printf(" % 02x: % 02x: % 02x: % 02x: % 02x: % 02x",
                    peer_addr[0], peer_addr[1], peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5]);

      esp_err_t result = esp_now_send(peer_addr, &data, 1);
    }
    if (result == ESP_OK) {
      actual = 1;
    }
  }
  return actual;
}
size_t ESPNow::write(const uint8_t *buffer, size_t size) {
  int remain = size;
  size_t actual = 0;
  //LINE
  esp_err_t result = !ESP_OK;
  if (manageSlave()) {
    if (size > ESP_NOW_MAX_DATA_LEN) {
      size = ESP_NOW_MAX_DATA_LEN;
    }
    //LINE
    const uint8_t *peer_addr = slave.peer_addr;
    Serial.printf( "%02x:%02x:%02x:%02x:%02x:%02x",
                   peer_addr[0], peer_addr[1], peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5]);
    //BREAK
    if (slaveFound) {
      //LINE
      while (remain > 0 && result != ESP_OK) {
        result = esp_now_send(peer_addr, buffer, size);
        //LINE
        if (result == ESP_OK) {
          //LINE
          remain -= size;
          actual += size;
          if (remain > 0) {
            result = !ESP_OK;
          }
        }
      }
    }
    if (result == ESP_OK) {
      //DEBUGOUT(actual)
      //DEBUGOUT("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      //DEBUGOUT("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      //DEBUGOUT("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      //DEBUGOUT("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      //DEBUGOUT("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      //DEBUGOUT("Peer not found.");
    } else {
      //DEBUGOUT("Not sure what happened");
    }
  }
  return actual;
}

bool ESPNow::manageSlave() {
  if (masterOrSlave == ESP_NOW_MASTER) {
    if (slave.channel == CHANNEL) {
      if (DELETEBEFOREPAIR) {
        deletePeer();
      }

      //Serial.print("Slave Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(slave.peer_addr);
      if ( exists) {
        // Slave already paired.
        //DEBUGOUT("Already Paired");
        return true;
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&slave);
        if (addStatus == ESP_OK) {
          // Pair success
          //DEBUGOUT("Pair success");
          return true;
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          //DEBUGOUT("ESPNOW Not Init");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          //DEBUGOUT("Invalid Argument");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          //DEBUGOUT("Peer list full");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          //DEBUGOUT("Out of memory");
          return false;
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          //DEBUGOUT("Peer Exists");
          return true;
        } else {
          //DEBUGOUT("Not sure what happened");
          return false;
        }
      }
    } else {
      // No slave found to process
      //DEBUGOUT("No Slave found to process");
      return false;
    }
  }
}

void ESPNow::deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  //Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    //Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    //DEBUGOUT("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    //DEBUGOUT("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    //DEBUGOUT("Peer not found.");
  } else {
    //DEBUGOUT("Not sure what happened");
  }
}

void ESPNow::OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), " % 02x: % 02x: % 02x: % 02x: % 02x: % 02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //DEBUGOUT("Last Packet Sent to: "); Serial.println(macStr);
  //DEBUGOUT("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}





void ESPNow::configDeviceAP(String mySSID) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  String uinqSSID = newUniqSSID(mySSID);
  bool result = WiFi.softAP(uinqSSID.c_str(), passWord, CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(mySSID));
  }
}

String ESPNow::newUniqSSID(String orignalSSID) {
  int i;
  int j;
  String UniqSSID;
  if (orignalSSID.length() >= 29) {
    //DEBUGOUT("orignalSSID is to long")
    //DEBUGSTOP;
  }
  int8_t scanResults = WiFi.scanNetworks();
  for (j = 1; j <= ESP_NOW_MAX_TOTAL_PEER_NUM; j++) {
    UniqSSID = orignalSSID + "_";
    if (j < 10) {
      UniqSSID += "0";
    }
    UniqSSID += String(j);
    for ( i = 0; i < scanResults; ++i) {
      String SSID = WiFi.SSID(i);
      if (UniqSSID == SSID) {
        break;
      }
    }
    if (i == scanResults) {
      return UniqSSID;
    }
  }
}
