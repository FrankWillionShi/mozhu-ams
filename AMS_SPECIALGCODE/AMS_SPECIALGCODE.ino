#define ASYNC_TCP_SSL_ENABLED 1
#define MQTT_SECURE 1
#include "defines.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <AsyncMQTT_ESP32.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}



//*******************Global Parameters begin********************
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
int Filament_Now = 0;
int FilaMent_Next = -1;
uint8_t workstate = 0;
bool Buffer_Func_Flag = 0;
bool ams_require_received = 0;
const std::string bambu_resume = R"({"print":{"command":"resume","sequence_id":"1"},"user_id":"1"})";
const std::string bambu_unload = R"({"print":{"command":"ams_change_filament","curr_temp":220,"sequence_id":"1","tar_temp":220,"target":255},"user_id":"1"})";
const std::string bambu_load = R"({"print":{"command":"ams_change_filament","curr_temp":220,"sequence_id":"1","tar_temp":220,"target":254},"user_id":"1"})";
const std::string bambu_done = R"({"print":{"command":"ams_control","param":"done","sequence_id":"1"},"user_id":"1"})";
bool mqttDataReadyFlag = 0;
JsonDocument jsonRecv;
//*******************Global Parameters end*********************

//*******************private functions begin*******************
void systeminit();
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void printSeparationLine();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(const uint16_t& packetId, const uint8_t& qos);
void onMqttUnsubscribe(const uint16_t& packetId);
void onMqttMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index, const size_t& total);
void onMqttPublish(const uint16_t& packetId);
void filament_buffer_func();
bool checkPauseFlag(JsonDocument& doc);
bool checkNextFilament(JsonDocument& doc,int& nextfilament);
void coreThread();
//*******************private functions end*********************

void setup()
{
  systeminit();
}

void loop()
{
  coreThread();
}

void systeminit()
{
  // init string
  Listen_Topic = std::string("device/") + BambuLab_Serial + Listen_Topic;
  Public_Topic = std::string("device/") + BambuLab_Serial + Public_Topic;

  // init serial and print version data
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(500);

  Serial.print("\nStarting mozhu-AMS on ");
  Serial.println(ARDUINO_BOARD);
  Serial.println(ASYNC_MQTT_ESP32_VERSION);

  // init timer
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  //init callback
  WiFi.onEvent(WiFiEvent);

  mqttClient.setCredentials(username,password);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  //init server
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  mqttClient.setSecure(MQTT_SECURE);

  if (MQTT_SECURE)
  {
    //mqttClient.addServerFingerprint((const uint8_t[])MQTT_SERVER_FINGERPRINT);
    mqttClient.addServerFingerprint((const uint8_t *)MQTT_SERVER_FINGERPRINT);
  }

  connectToWifi();

  //init GPIO
  for(int i = 0; i < 4; i++)
  {
    pinMode(Motor_Forward[i],OUTPUT);
    digitalWrite(Motor_Forward[i],0);
    pinMode(Motor_Bacword[i],OUTPUT);
    digitalWrite(Motor_Bacword[i],0);
    pinMode(Filament_detection[i],INPUT_PULLDOWN);
  }
  pinMode(Buffer_Detection[0],INPUT_PULLDOWN);
  pinMode(Buffer_Detection[1],INPUT_PULLDOWN);
  Serial.println("GPIO init complete\n");

  //init filament
  if(!JUMP_FILAMENT_INIT)
  {
    if(Filament_Num > 4)
      Filament_Num = 4;
    if(Filament_Num < 1)
      Filament_Num = 1;
    for(int i = 0; i < Filament_Num; i++)
    {
      //make sure the filament is installed correctly
      Serial.print("Filament No.");
      Serial.print(i);
      Serial.println("input");
      digitalWrite(Motor_Forward[i], 1);
      while(!digitalRead(Filament_detection[i])) {};
      Serial.print("Filament No.");
      Serial.print(i);
      Serial.println("output");
      digitalWrite(Motor_Forward[i],0);
      digitalWrite(Motor_Bacword[i],1);
      while(digitalRead(Filament_detection[i])) {};
      digitalWrite(Motor_Bacword[i],0);
      Serial.print("Filament No.");
      Serial.print(i);
      Serial.println("complete");
    }
    Serial.println("filament init complete\n");
  }

}

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
#if USING_CORE_ESP32_CORE_V200_PLUS

    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi ready");
      break;

    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi STA starting");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi STA connected");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("WiFi lost IP");
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
#else

    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
#endif

    default:
      break;
  }
}

void printSeparationLine()
{
  Serial.println("************************************************");
}

void onMqttConnect(bool sessionPresent)
{
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);
  Serial.print("PubTopic: ");
  Serial.println(Listen_Topic.c_str());

  printSeparationLine();
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(Listen_Topic.c_str(), 0);
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);

  printSeparationLine();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  (void) reason;

  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(const uint16_t& packetId, const uint8_t& qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(const uint16_t& packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index, const size_t& total)
{
  if(SHOW_RECEIVE_LOG)
  {
    Serial.println("Publish received.");
    Serial.print("  topic: ");
    Serial.println(topic);
    Serial.print("  qos: ");
    Serial.println(properties.qos);
    Serial.print("  dup: ");
    Serial.println(properties.dup);
    Serial.print("  retain: ");
    Serial.println(properties.retain);
    Serial.print("  len: ");
    Serial.println(len);
    Serial.print("  index: ");
    Serial.println(index);
    Serial.print("  total: ");
    Serial.println(total);
  }

  if(SHOW_RECEIVE_DETAIL)
  {
    Serial.println(payload);
  }
  
  if(!mqttDataReadyFlag)
  {
    deserializeJson(jsonRecv,(char*)payload,len);
  }
  mqttDataReadyFlag = 1;
}

void onMqttPublish(const uint16_t& packetId)
{
  Serial.println("Publish acknowledged");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void filament_buffer_func()
{
  if(workstate == 0)
  {
    if(digitalRead(Buffer_Detection[1]))
    {
      Buffer_Func_Flag = 1;
      digitalWrite(Motor_Forward[Filament_Now], 1);
    }
    if(digitalRead(Buffer_Detection[0]) == 1 && Buffer_Func_Flag == 1)
    {
      digitalWrite(Motor_Forward[Filament_Now], 0);
      Buffer_Func_Flag = 0;
    }
  }
}


bool checkPauseFlag(JsonDocument& doc)
{
  JsonObject print = doc["print"];
  const char* print_gcode_state = print["gcode_state"]; // "PAUSE"
  if(print_gcode_state == nullptr)
    return 0;
  std::string gcode_state = std::string(print_gcode_state);
  if(gcode_state == "PAUSE")
    return 1;
  else
    return 0;
}

bool checkNextFilament(JsonDocument& doc,int& nextfilament)
{
  JsonObject print = doc["print"];

  int print_mc_percent = print["mc_percent"]; // 101 + next filament
  if(print_mc_percent > 100)
  {
    nextfilament = print_mc_percent - 101;
    return 1;
  }
  else
  {
    return 0;
  }
}

void coreThread()
{  
  //workstate:
  //0: standby
  //1: AMS unloading
  //2: AMS loading


  // filament buffer function
  filament_buffer_func();

  checkNextFilament(jsonRecv,FilaMent_Next)

  // AMS FUNCTION
  //if no MQTT data receive, skip this function
  if(mqttDataReadyFlag && workstate == 0)
  {
    if(checkPauseFlag(jsonRecv))
    {
      if(Buffer_Func_Flag)
      {
        //if the filament buffer function enable motor running, stop the motor and clear flag
        Serial.printf("filament change require received, start unloading filament\nfilament now is %d, filament next is %d \n",Filament_Now,FilaMent_Next);
        digitalWrite(Motor_Forward[Filament_Now],0);
        workstate = 1;
        Buffer_Func_Flag = 0;
      }
      // enable motor to unload filament
      digitalWrite(Motor_Bacword[Filament_Now],1);
      mqttDataReadyFlag = 0;
    }
  }
  else if(workstate == 1)
  {
    // wait until filament is unload into AMS
    if(digitalRead(Filament_detection[Filament_Now]) == 0)
    {
      Serial.printf("filament unload complete, start load new filament\n");
      // if the filament switch is off,it means the filament is back into AMS successfully
      // stop unload motor
      digitalWrite(Motor_Bacword[Filament_Now],0);
      Filament_Now = -1;
      // enable motor to send new filament into printer 
      digitalWrite(Motor_Forward[FilaMent_Next],1);
      workstate = 2;
    }
  }
  else if(workstate == 2)
  {
    // wait until filament buffer is full
    // Warning: When the resistance along the path of consumable filament to the printer is too high, it may cause the feeding to be prematurely stopped
    if(digitalRead(Buffer_Detection[0]) == 1)
    {
      Serial.printf("filament load complete, sendind command to printer\n");
      digitalWrite(Motor_Forward[FilaMent_Next], 0);
      // filament change is complete, send command to printer and updata filament data
      Filament_Now = FilaMent_Next;
      FilaMent_Next = -1;
    }
  }
}