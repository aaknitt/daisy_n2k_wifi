/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
// Reads AIVDM messages from NMEA0183 (ESP32 UART 2 on GPIO 16) and forwards them to the N2k bus
// Version 0.7, 13.01.2022, AK-Homberger

// Is using modified (clang#14 to clang#11) version of this AIS decoder: https://github.com/aduvenhage/ais-decoder
// AIS decoder is under MIT license: https://github.com/aduvenhage/ais-decoder/blob/master/LICENSE

/* Modified NMEA2000_CAN.h to use jiauka repo that supports ESP32C3 instead of packaged ttlappalainen repo
#elif USE_N2K_CAN == USE_N2K_ESP32_CAN
// #include <NMEA2000_esp32.h>       // https://github.com/ttlappalainen/NMEA2000_esp32
//tNMEA2000 &NMEA2000=*(new tNMEA2000_esp32());
#include <NMEA2000_esp32xx.h>     // https://github.com/jiauka/NMEA2000_esp32xx
tNMEA2000 &NMEA2000=*(new tNMEA2000_esp32xx());
*/

// Currently following AIS message types are supported: 1-3, 5, 14, 18, 19, 24A, 24B

#define ESP32_CAN_TX_PIN GPIO_NUM_6 //GPIO6 => D4  Set CAN TX pin to D4 
#define ESP32_CAN_RX_PIN GPIO_NUM_7 //GPIO7 => D5  Set CAN RX pin to D5

#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183.h>
#include <Preferences.h>
#include <HardwareSerial.h>

#include "NMEA0183AIStoNMEA2000.h"  // Contains class, global variables and code !!!

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include "AsyncUDP.h"
#include "FS.h"
#include <LittleFS.h>
#include <HardwareSerial.h>

#define MAX_NMEA0183_MESSAGE_SIZE 150
#define FORMAT_LITTLEFS_IF_FAILED true

const char *soft_ap_ssid = "dAISy";

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";
const char* PARAM_INPUT_5 = "apssid";
const char* PARAM_INPUT_6 = "appass";
const char* PARAM_INPUT_7 = "nmea_udp_port";
const char* PARAM_INPUT_8 = "nmea_udp_enabled";

//Variables to save values from HTML forms
std::string ssid;
std::string pass;
std::string ip;
std::string gateway;
std::string apssid;
std::string appass;
std::string nmea_udp_enabled;
std::string nmea_udp_port;
std::string fw_version;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";
const char* apssidPath = "/apssid.txt";
const char* appassPath = "/appass.txt";
const char* logPath = "/log.txt";
const char* nmeaUdpPortPath = "/nmea_udp_port.txt";
const char* nmeaUdpEnabledPath = "/nmea_udp_enabled.txt";
const char* fwVersionPath = "/firmware.txt";

DNSServer dnsServer;
AsyncWebServer server(80);
AsyncUDP udp;

// NMEA message and stream for AIS receiving
tNMEA0183Msg NMEA0183Msg;
tNMEA0183 NMEA0183;

int NodeAddress;                    // To store last Node Address
Preferences preferences;            // Nonvolatile storage on ESP32 - To store LastDeviceAddress

MyAisDecoder decoder;               // Create decoder object
AIS::DefaultSentenceParser parser;  // Create parser object

HardwareSerial MySerial0(0);

//*****************************************************************************
void setup() {
  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0;

  Serial.begin(115200);
  delay(1000);
  Serial.println("starting up");

  if(!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED,"/data",255,"spiffs")){
        Serial.println("LittleFS Mount Failed");
        return;
    }
  else{
    Serial.println("LittleFS Mount OK");
    listDir(LittleFS, "/", 3);
  }
  ssid = readFile(LittleFS, ssidPath);
  pass = readFile(LittleFS, passPath);
  ip = readFile(LittleFS, ipPath);
  gateway = readFile(LittleFS, gatewayPath);
  apssid = readFile(LittleFS, apssidPath);
  appass = readFile(LittleFS, appassPath);
  nmea_udp_enabled = readFile(LittleFS,nmeaUdpEnabledPath); 
  nmea_udp_port = readFile(LittleFS,nmeaUdpPortPath);
  fw_version = readFile(LittleFS,fwVersionPath);
  Serial.print("UDP port: ");
  Serial.println(nmea_udp_port.c_str());

  //start up our WiFi access point
  if (apssid.c_str() == ""){
    WiFi.softAP(soft_ap_ssid, NULL);
  }
  else{
    WiFi.softAP(apssid.c_str(), appass.c_str());
  } 
  Serial.println("Starting DNS Server");
  dnsServer.start(53, "*", WiFi.softAPIP());

  //connect to WiFi access point as a client
  if(ssid!=""){
    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.onEvent(OnWiFiEvent);
    if(ip!=""){
      IPAddress localIP;
      IPAddress localGateway;
      IPAddress subnet(255, 255, 0, 0);
      localIP.fromString(ip.c_str());
      localGateway.fromString(gateway.c_str());
      WiFi.config(localIP, localGateway, subnet);
    }
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.print("Connecting to WiFi SSID ");
    Serial.println(ssid.c_str());
        
    Serial.println("Waiting 2 seconds");
    delay(2000);
  }

  //set up web server
  setupServer();
  Serial.println("beginning server");
  server.begin();
  
  // Serial2.begin(38400, SERIAL_8N1);   // Configure Serial2 (GPIO 16)
  NMEA0183.Begin(&MySerial0, 3, 38400); // Start NMEA0183 stream handling

  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

  // Setup NMEA2000 system
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 10,  // Manufacturer's product code
                                 "dAISy2+ AIS N2k",  // Manufacturer's Model ID
                                 fw_version.c_str(), // Manufacturer's Software version code
                                 "1.0.0"  // Manufacturer's Model version
                                );
  // Det device information
  NMEA2000.SetDeviceInformation(id,  // Unique number. Use e.g. Serial number.
                                195, // Device function=AIS. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                60,  // Device class=Navigation. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 32);  // Read stored last NodeAddress, default 32
  preferences.end();

  Serial.printf("NodeAddress=%d\n", NodeAddress);

  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(MyHandleNMEA2000Msg);

  NMEA2000.Open();
  Serial.println("setup complete");
}


//*****************************************************************************
void HandleSytemTime(const tN2kMsg & N2kMsg) {
  unsigned char SID;
  tN2kTimeSource TimeSource;
  double SecondsSinceMidnight = 0;

  ParseN2kSystemTime(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight, TimeSource);
}


//*****************************************************************************
void HandleGNSS(const tN2kMsg & N2kMsg) {

  unsigned char SID;
  double Latitude;
  double Longitude;
  double Altitude;
  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;
  unsigned char nSatellites;
  double HDOP;
  double PDOP;
  double GeoidalSeparation;
  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceSationID;
  double AgeOfCorrection;
  double SecondsSinceMidnight = 0;

  ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight, Latitude, Longitude, Altitude,
               GNSStype, GNSSmethod, nSatellites, HDOP, PDOP, GeoidalSeparation,
               nReferenceStations, ReferenceStationType, ReferenceSationID, AgeOfCorrection);
}


//*****************************************************************************
void MyHandleNMEA2000Msg(const tN2kMsg &N2kMsg) {

  if (N2kMsg.PGN == 129029UL) HandleGNSS(N2kMsg);      // Just to get time from GNSS
  if (N2kMsg.PGN == 126992UL) HandleSytemTime(N2kMsg); // or this way
}


//*****************************************************************************
void CheckSourceAddressChange() {
  int SourceAddress = NMEA2000.GetN2kSource();
  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}


//*****************************************************************************
void ParseAIVDM_Message() {
  int i = 0;
  char buf[MAX_NMEA0183_MESSAGE_SIZE];

  if (!NMEA0183.GetMessage(NMEA0183Msg)) return;  // New message? If not return
  
  // Select (comment/uncomment) if you want to decode only other ship (AIVDM) or also own ship (AIVDO) messages
  
  // if (!NMEA0183Msg.IsMessageCode("VDM") && !NMEA0183Msg.IsMessageCode("VDO")) return;   // Not a AIVDM/AIVDO message, return
  if (!NMEA0183Msg.IsMessageCode("VDM")) return;   // Not a AIVDM message, return

  if (!NMEA0183Msg.GetMessage(buf, MAX_NMEA0183_MESSAGE_SIZE)) return;  // GetMessage copy to buffer failed

  strcat(buf, "\n");  // Decoder expects that.

  //Forward NMEA0183 sentence to UDP port, if enabled
  int len = strlen(buf);
  if (strcmp(nmea_udp_enabled.c_str(),"true")==0){
    IPAddress ap_bcst_ip = WiFi.softAPIP();
    ap_bcst_ip[3] = 255;
    udp.writeTo(reinterpret_cast<const uint8_t*>(buf),len,ap_bcst_ip,atoi(nmea_udp_port.c_str()));
    if (WiFi.localIP() != IPAddress(0,0,0,0)){
        IPAddress lcl_bcst_ip = WiFi.localIP();
        lcl_bcst_ip[3] = 255;
        udp.writeTo(reinterpret_cast<const uint8_t*>(buf),len,lcl_bcst_ip,atoi(nmea_udp_port.c_str()));
    }
  }
  
  do {
    i = decoder.decodeMsg(buf, strlen(buf), i, parser);   // Decode AIVDM/AIVDO message
  } while (i != 0);                                       // To be called until return value is 0
  Serial.println("msg handled");
}


//*****************************************************************************
void loop() {
  NMEA2000.ParseMessages();
  ParseAIVDM_Message();      // Parse AIS
  CheckSourceAddressChange();
}

void setupServer(){
  server.on("/hello", HTTP_GET, [](AsyncWebServerRequest * request) {
    Serial.println("got a request on /hello");
    if (ON_STA_FILTER(request)) {
      request->send(200, "text/plain", "Hello from STA");
      return;
 
    } else if (ON_AP_FILTER(request)) {
      request->send(200, "text/plain", "Hello from AP");
      return;
    }
    request->send(200, "text/plain", "Hello from undefined");
  });
  server.on("/ssid", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", ssid.c_str());
  });
  server.on("/pw", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", pass.c_str());
  });
  server.on("/ip", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", ip.c_str());
  });
  server.on("/gw", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", gateway.c_str());
  });
  server.on("/nmeaUdpEnabled", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", nmea_udp_enabled.c_str());
  });
  server.on("/nmeaUdpPort", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", nmea_udp_port.c_str());
  });
  server.on("/apssid", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (apssid.c_str()!=""){
      request->send(200, "text/plain", apssid.c_str());
    }
    else{
      request->send(200, "text/plain", soft_ap_ssid);
    }
  });
  server.on("/appass", HTTP_GET, [](AsyncWebServerRequest * request) {
    if (apssid.c_str()!=""){
      request->send(200, "text/plain", appass.c_str());
    }
    else{
      request->send(200, "text/plain", "");
    }
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(LittleFS, "/wifimanager.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/");
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      uint8_t restart = 0;
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid.c_str());
            // Write file to save value
            writeFile(LittleFS, ssidPath, ssid.c_str());
            restart = 1;
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass.c_str());
            // Write file to save value
            writeFile(LittleFS, passPath, pass.c_str());
            restart = 1;
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip.c_str());
            // Write file to save value
            writeFile(LittleFS, ipPath, ip.c_str());
            restart = 1;
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway.c_str());
            // Write file to save value
            writeFile(LittleFS, gatewayPath, gateway.c_str());
            restart = 1;
          }
          // HTTP POST apssid value
          if (p->name() == PARAM_INPUT_5) {
            apssid = p->value().c_str();
            Serial.print("AP SSID set to: ");
            Serial.println(apssid.c_str());
            // Write file to save value
            writeFile(LittleFS, apssidPath, apssid.c_str());
            restart = 1;
          }
          // HTTP POST appass value
          if (p->name() == PARAM_INPUT_6) {
            appass = p->value().c_str();
            Serial.print("AP PW set to: ");
            Serial.println(appass.c_str());
            // Write file to save value
            writeFile(LittleFS, appassPath, appass.c_str());
            restart = 1;
          }
          // HTTP POST NMEA UDP port value
          if (p->name() == PARAM_INPUT_7) {
            nmea_udp_port = p->value().c_str();
            Serial.print("NMEA UDP Port set to: ");
            Serial.println(nmea_udp_port.c_str());
            // Write file to save value
            writeFile(LittleFS, nmeaUdpPortPath, nmea_udp_port.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
        // HTTP POST NMEA Forward setting value
        if (p->name() == PARAM_INPUT_8) {
          nmea_udp_enabled = p->value().c_str();
          Serial.print("NMEA forwarding set to: ");
          Serial.println(nmea_udp_enabled.c_str());
          // Write file to save value
          writeFile(LittleFS, nmeaUdpEnabledPath, nmea_udp_enabled.c_str());
        }
      }
      if(restart==1){
        request->send(200, "text/plain", "Done. WiFi will restart and attempt to connect to your router");
        delay(3000);
        ESP.restart();
      }
      else{
        //request->send(200, "text/plain", "Settings Saved!");
        request->redirect("/");
      }
  });
}

void OnWiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 Connected to WiFi Access Point ");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.print("ESP32 soft AP started with IP ");
      Serial.println(WiFi.softAPIP());
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("Station connected to ESP32 soft AP");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("Station disconnected from ESP32 soft AP");
      break;
    default: break;
  }
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

std::string readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    size_t filesize = file.size();
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return "";
    }

    Serial.println("- read from file:");
    String data = file.readString();
    file.close();
    return std::string(data.c_str(),data.length());
}

void printFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    size_t filesize = file.size();
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    String data = file.readString();
    Serial.print(data);
    file.close();
    return;
}
