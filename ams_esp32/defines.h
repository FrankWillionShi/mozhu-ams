#ifndef MOZHU_AMS_SETTINGS
#define MOZHU_AMS_SETTINGS

#define _ASYNC_MQTT_LOGLEVEL_               1
#include <string.h>

// *****************User Settings begin***********************

#define WIFI_SSID         "robomaster109"
#define WIFI_PASSWORD     "robo123456"
#define MQTT_HOST         IPAddress(192, 168, 0, 100)
const char* password = "24282340";
std::string BambuLab_Serial = "01S00C351400077";
int Filament_Num = 2;
// *****************User Settings end***********************


// ****************devoifault settings end*********************
// Warning: do not edit these settings if not necessary
const uint8_t MQTT_SERVER_FINGERPRINT[] = {0x2E, 0xA1, 0xA1, 0x8B, 0x61, 0x42, 0xDB, 0x96, 0x8F, 0x41, 0x4D, 0x4B, 0x7D, 0xAF, 0x9A, 0x64, 0xD3, 0x3A, 0x42, 0x07};
const char* username = "bblp";
const int Motor_Forward[4] = {0,2,10,7};
const int Motor_Bacword[4] = {1,3,6,11};
const int Filament_detection[4] = {5,4,8,9};
const int Buffer_Detection[2] = {12,13};
std::string Listen_Topic = "/report";
std::string Public_Topic = "/request";
#define MQTT_PORT       8883
// ****************default settings end*********************

#endif    //MOZHU_AMS_SETTINGS
