#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>
#include <WiFiClientSecure.h>

//*********************User Settings**********************
const char* WIFI_SSID = "robomaster109";
const char* WIFI_PASSWORD = "robo123456";
const char* BambuLab_IP = "192.168.0.100";
const char* BambuLab_Password = "24282340";
std::string BambuLab_Serial = "01S00C351400077";
const int Motor_Forward[4] = {0,2,10,7};
const int Motor_Bacword[4] = {1,3,6,11};
const int Filament_detection[4] = {5,4,8,9};
const int Buffer_Detection[2] = {12,13};
const float Filament_K_Val[4] = {0.030,0.030,0.030,0.030};
int Filament_Num = 2;
//********************************************************

//*******************Global Parameters********************
WiFiClientSecure espClient;
PubSubClient mqttClient;
const char* BambuLab_Username = "bblp";
const int BambuLab_Port = 8883;
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

const char *ca_cert0 = R"EOF(
-----BEGIN CERTIFICATE-----
MIIC2DCCAcACCQD7SqTgTj31/DANBgkqhkiG9w0BAQsFADBCMQswCQYDVQQGEwJD
TjEiMCAGA1UECgwZQkJMIFRlY2hub2xvZ2llcyBDby4sIEx0ZDEPMA0GA1UEAwwG
QkJMIENBMB4XDTIzMDUxOTA1MjYzMFoXDTMzMDUxNjA1MjYzMFowGjEYMBYGA1UE
AwwPMDFTMDBDMzUxNDAwMDc3MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKC
AQEAuWe/nNHQwGFBoK0jsDHPnMRrcC4/Vuy/TXvghiP61moCX7AQKboPyQDbaxSY
Q/u7XsUcY8U06AeuuFo/v1xSGAzO4FekUO8wD3utr8kHCIj7gbEre/ZkG2JJ6yaJ
jF+1PpcEvdZ1V4eIerN7ahv5DUALyxY59BP/Bbhrd3KxJ3UF/FSbd8TtKxjAEqi7
GPg8+uD7RqxFo+j3OMvGXDE4iVhIWkFcRy9WnO5roV1cv0TEVlS53zX9T9D0YKkX
v5U6PTbA2myIVB8ztX+Wi+Gx3UCcPGHewa3yEAuqGj+V8xe5NDl5s31l2TLdvRVj
8LloygZMVs/mk79yzueCyoFrJwIDAQABMA0GCSqGSIb3DQEBCwUAA4IBAQCuM+3n
7/A9RnVTfO1qGNElLRf+WHEstFjln6rfEhgvq4RjrdAA51VGI6qusFA9eNIRxQKt
8FSfqeyU0KL+9Q5EH+AmsbeC2qWLEIw2n+9/PGxE0sU3G7Fgl39hobUoKb1kUw/5
cX3yAzVyIx154Qt8PSoG6Ts9YWKPjRSHjR7RKU0lQA6dKv/tPlTU3T9tY0Uyyfsy
A30mBcm359NyE98xZwFW6zNyPEtljQh89YWkMhgWey59KqoVYGGJe+581M1fQ4ex
LNJ4PDwNRi28rNSwVCOemVcWdFk681CeIl9qfRprPb9hsD3DDsa0bOh2dx1+h/zx
BtoNLOi9aIcdMYHW
-----END CERTIFICATE-----
)EOF";

const char *ca_cert1 = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDZTCCAk2gAwIBAgIUV1FckwXElyek1onFnQ9kL7Bk4N8wDQYJKoZIhvcNAQEL
BQAwQjELMAkGA1UEBhMCQ04xIjAgBgNVBAoMGUJCTCBUZWNobm9sb2dpZXMgQ28u
LCBMdGQxDzANBgNVBAMMBkJCTCBDQTAeFw0yMjA0MDQwMzQyMTFaFw0zMjA0MDEw
MzQyMTFaMEIxCzAJBgNVBAYTAkNOMSIwIAYDVQQKDBlCQkwgVGVjaG5vbG9naWVz
IENvLiwgTHRkMQ8wDQYDVQQDDAZCQkwgQ0EwggEiMA0GCSqGSIb3DQEBAQUAA4IB
DwAwggEKAoIBAQDL3pnDdxGOk5Z6vugiT4dpM0ju+3Xatxz09UY7mbj4tkIdby4H
oeEdiYSZjc5LJngJuCHwtEbBJt1BriRdSVrF6M9D2UaBDyamEo0dxwSaVxZiDVWC
eeCPdELpFZdEhSNTaT4O7zgvcnFsfHMa/0vMAkvE7i0qp3mjEzYLfz60axcDoJLk
p7n6xKXI+cJbA4IlToFjpSldPmC+ynOo7YAOsXt7AYKY6Glz0BwUVzSJxU+/+VFy
/QrmYGNwlrQtdREHeRi0SNK32x1+bOndfJP0sojuIrDjKsdCLye5CSZIvqnbowwW
1jRwZgTBR29Zp2nzCoxJYcU9TSQp/4KZuWNVAgMBAAGjUzBRMB0GA1UdDgQWBBSP
NEJo3GdOj8QinsV8SeWr3US+HjAfBgNVHSMEGDAWgBSPNEJo3GdOj8QinsV8SeWr
3US+HjAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3DQEBCwUAA4IBAQABlBIT5ZeG
fgcK1LOh1CN9sTzxMCLbtTPFF1NGGA13mApu6j1h5YELbSKcUqfXzMnVeAb06Htu
3CoCoe+wj7LONTFO++vBm2/if6Jt/DUw1CAEcNyqeh6ES0NX8LJRVSe0qdTxPJuA
BdOoo96iX89rRPoxeed1cpq5hZwbeka3+CJGV76itWp35Up5rmmUqrlyQOr/Wax6
itosIzG0MfhgUzU51A2P/hSnD3NDMXv+wUY/AvqgIL7u7fbDKnku1GzEKIkfH8hm
Rs6d8SCU89xyrwzQ0PR853irHas3WrHVqab3P+qNwR0YirL0Qk7Xt/q3O1griNg2
Blbjg3obpHo9
-----END CERTIFICATE-----

)EOF";

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
  //Serial.println("test");
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
  espClient.setCACert(ca_cert1);
  const char* ClientID = "mzams";
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