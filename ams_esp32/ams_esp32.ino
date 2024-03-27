#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>

//*********************User Settings**********************
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";
const char* BambuLab_IP = "your device IP";
const char* BambuLab_Password = "your device password";
std::string BambuLab_Serial = "your device serial";
const int Motor_Forward[4] = {0,2,10,7};
const int Motor_Bacword[4] = {1,3,6,11};
const int Filament_detection[4] = {5,4,8,9};
const int Buffer_Detection[2] = {12,13};
const float Filament_K_Val[4] = {0.030,0.030,0.030,0.030};
int Filament_Num = 4;
//********************************************************

//*******************Global Parameters********************
WiFiClient espClient;
PubSubClient mqttClient;
const char* BambuLab_Username = "bblp";
const int BambuLab_Port = 3333;
std::string Listen_Topic = "/report";
std::string Public_Topic = "/request";
int Filament_Now = 0;
int FilaMent_Next = -1;
int workstate = 0;
bool Buffer_Func_Flag = 0;
bool DEBUG_MODE = 0;
bool ams_require_received = 0;
const std::string bambu_resume = R"({"print":{"command":"resume","sequence_id":"1"},"user_id":"1"})";
const std::string bambu_unload = R"({"print":{"command":"ams_change_filament","curr_temp":220,"sequence_id":"1","tar_temp":220,"target":255},"user_id":"1"})";
const std::string bambu_load = R"({"print":{"command":"ams_change_filament","curr_temp":220,"sequence_id":"1","tar_temp":220,"target":254},"user_id":"1"})";
const std::string bambu_done = R"('{"print":{"command":"ams_control","param":"done","sequence_id":"1"},"user_id":"1"})";
//********************************************************

void system_init();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void filament_buffer_func();
bool checkPauseFlag(JsonDocument& doc);
bool checkNextFilament(JsonDocument& doc,int& nextfilament);
bool checkHWSwitchState(JsonDocument& doc,bool& state);
bool checkAMSStatus(JsonDocument& doc,bool& status);

void setup() 
{
  Serial.begin(115200);
  Serial.println("system init\n");
  system_init();
  Serial.println("system init complete\n");
}

void loop() 
{
  filament_buffer_func();
  Serial.println("test");
}

void system_init()
{
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
  if(Filament_Num > 4)
    Filament_Num = 4;
  if(Filament_Num < 1)
    Filament_Num = 1;
  for(int i = 0; i < Filament_Num; i++)
  {
    //make sure the filament is installed correctly
    digitalWrite(Motor_Forward[i], 1);
    while(!digitalRead(Filament_detection[i])) {};
    digitalWrite(Motor_Forward[i],0);
    digitalWrite(Motor_Bacword[i],1);
    while(digitalRead(Filament_detection[i])) {};
    digitalWrite(Motor_Bacword[i],0);
  }
  Serial.println("filament init complete\n");

  //complete string
  Listen_Topic = std::string("device/") + BambuLab_Serial + Listen_Topic;
  Public_Topic = std::string("device/") + BambuLab_Serial + Public_Topic;
  
  //connect wifi
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }

  //set mqtt client and connect mqtt server
  const char* ClientID = "mozhu-AMS";
  mqttClient.setClient(espClient);
  mqttClient.setServer(BambuLab_IP,BambuLab_Port);
  mqttClient.setCallback(mqtt_callback);
  if(mqttClient.connect(ClientID,BambuLab_Username,BambuLab_Password))
    Serial.println("bambu connect success\n");
  else
    Serial.println("bambu connect fail\n");
  mqttClient.subscribe(Listen_Topic.c_str());
}

void filament_buffer_func()
{
  if(workstate == 0)
  {
    if(digitalRead(Buffer_Detection[1]))
    {
      Buffer_Func_Flag = 1;
      digitalWrite(Motor_Forward[Filament_Now], 1);
      while(!digitalRead(Buffer_Detection[0])) {};
      digitalWrite(Motor_Forward[Filament_Now], 0);
      Buffer_Func_Flag = 0;
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  JsonDocument jsonRecv;
  deserializeJson(jsonRecv,(char*)payload,length);

  if(checkNextFilament(jsonRecv,FilaMent_Next))
  {
    ams_require_received = 1;
    Serial.println("filament change require received\n");
  }
  if(workstate == 0)
  {
    std::string gcode_state = jsonRecv["print"]["gcode_state"];
    if (checkPauseFlag(jsonRecv))
    {
      Serial.println("printer pause command received\n");
      if(checkNextFilament(jsonRecv,FilaMent_Next) || ams_require_received)
      {
        ams_require_received = 0;
        Serial.println("filament change start\n");
        // start change filament
        if(FilaMent_Next == Filament_Now)
        {
          // next filament is the same with the current filament,there is no need to change filament
          Serial.println("next filament is the same with the current filament,there is no need to change filament\n");
          mqttClient.publish(Public_Topic.c_str(),bambu_resume.c_str(),bambu_resume.size());
          return;
        }
        //make sure the motor is not running
        if(Buffer_Func_Flag)
        {
          digitalWrite(Motor_Forward[Filament_Now],0);
          Buffer_Func_Flag = 0;
        }
        // unload the filament
        Serial.println("start filament unload\n");
        mqttClient.publish(Public_Topic.c_str(),bambu_unload.c_str(),bambu_unload.size());
        workstate += 1;
      }
    }
  }
  else if(workstate == 1)
  {
    bool switch_status = 1;
    if(checkHWSwitchState(jsonRecv,switch_status))
    {
      if(0 == switch_status)
      {
        Serial.println("filament unload start\n");
        digitalWrite(Motor_Bacword[Filament_Now],1);
        // after the filament is out of printer
        while(digitalRead(Filament_detection[Filament_Now]) == 1) {};// wait until filament is back into AMS
        Serial.println("filament unload complete\n");
        digitalWrite(Motor_Bacword[Filament_Now],0);
        Filament_Now = -1;
        // send new filament to printer
        Serial.println("filament load start\n");
        digitalWrite(Motor_Forward[FilaMent_Next],1);
        workstate += 1;
      }
    }
  }
  else if(workstate == 2)
  {
    bool switch_status = 0;
    if(checkHWSwitchState(jsonRecv,switch_status))
    {
      if(1 == switch_status)
      {
        // new filament is in posion and ready to load
        digitalWrite(Motor_Forward[FilaMent_Next],0);
        Serial.println("filament load complete\n");
        mqttClient.publish(Public_Topic.c_str(),bambu_load.c_str(),bambu_load.size());
        delay(10000);
        Serial.println("filament load command sent\n");
        mqttClient.publish(Public_Topic.c_str(),bambu_done.c_str(),bambu_done.size());
        workstate += 1;
      }
    }
  }
  else if(workstate == 3)
  {
    bool AMSStatus = 0;
    if(checkAMSStatus(jsonRecv,AMSStatus))
    {
      if(AMSStatus)
      {
        // filament wash complete, continue print
        Serial.println("AMS function complete\n");
        mqttClient.publish(Public_Topic.c_str(),bambu_resume.c_str(),bambu_resume.size());
        workstate = 0;
      }
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
  int print_mc_percent = print["mc_percent"]; // 101
  int print_mc_remaining_time = print["mc_remaining_time"]; // 1
  if(print_mc_remaining_time != 101)
    return 0;
  nextfilament = print_mc_remaining_time;
  return 1;
}

bool checkHWSwitchState(JsonDocument& doc,bool& state)
{
  JsonObject print = doc["print"];
  const char* state_exist = print["hw_switch_state"];
  if(state_exist == nullptr)
  return 0;
  int print_hw_switch_state = print["hw_switch_state"]; // 0
  if(print_hw_switch_state == 0)
    state = 0;
  else
    state = 1;
  return 1;
}

bool checkAMSStatus(JsonDocument& doc,bool& status)
{
  JsonObject print = doc["print"];
  JsonObject print_ams = print["ams"];
  const char* print_ams_tray_pre = print_ams["tray_pre"];
  if(print_ams_tray_pre == nullptr)
    return 0;
  std::string tray_pre = std::string(print_ams_tray_pre);
  if(tray_pre == "254")
    status = 1;
  else
    status = 0;
  return 1;
}