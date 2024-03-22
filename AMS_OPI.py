import paho.mqtt.client as mqtt
from enum import Enum
import wiringpi
from wiringpi import GPIO
import ssl
import json
import socket
import time

DEBUG = False

################## 以下为用户配置区 ##################
MQTT_SERVER = "ip_address"  # 将ip_address替换为打印机的IP地址
PASSWORD = "your_password"  # 将your_password替换为局域网模式里的密码
DEVICE_SERIAL = "your_device_serial"  # 将your_device_serial替换为设备的序列号
CH_MAP = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]  # 通道映射表
TCP_SERVER = "ip_address"  # 将ip_address替换为驱动板的IP地址
FILAMENT_K_VAL = [0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030,0.030] #先放16个槽位的耗材信息在这里
MOTOR_FORWARD = [0,1,2,3]   #对应电机正转的GPIO引脚
MOTOR_BACKWARD = [4,5,6,7]  #对应电机反转的GPIO引脚
FILAMENT_DETECTION = [8,9,10,11]    #对应耗材断料检测引脚
BUFFER_DETECTION = [12,13]  #进料缓冲器检测引脚
################## 以上为用户配置区 ##################

class status(Enum):
    standby = 0
    bambuOffline = -1
    unloadingFilament = 1
    AMSloadingFilament = 2
    bambuLoadingFilament = 3
    bambuWashFilament = 4

# 定义服务器信息和认证信息
TCP_PORT = 3333
MQTT_PORT = 8883
MQTT_VERSION = mqtt.MQTTv311
USERNAME = "bblp"

# 订阅和发送的主题
TOPIC_SUBSCRIBE = f"device/{DEVICE_SERIAL}/report"
TOPIC_PUBLISH = f"device/{DEVICE_SERIAL}/request"

############## 全局变量区 #######################
bambu_flament_detection_flag = True
AMS_status = status.bambuOffline
now_filament = 0
next_filament = 0
################################################


bambu_resume = '{"print":{"command":"resume","sequence_id":"1"},"user_id":"1"}'
bambu_unload = '{"print":{"command":"ams_change_filament","curr_temp":220,"sequence_id":"1","tar_temp":220,"target":255},"user_id":"1"}'
bambu_load = '{"print":{"command":"ams_change_filament","curr_temp":220,"sequence_id":"1","tar_temp":220,"target":254},"user_id":"1"}'
bambu_done = '{"print":{"command":"ams_control","param":"done","sequence_id":"1"},"user_id":"1"}'

# 判断进料操作是否完成
def BambuFilamentReady():
    return True

# 向发印记发送耗材校准数据
def sendCaliData(client, filament_num):
    k_value = FILAMENT_K_VAL[filament_num]
    data2send = f'{{"print": {{"command": "extrusion_cali_set", "k_value": {k_value}, "n_coef": 1.399999976158142, "tray_id": 254, "sequence_id": "1"}}, "user_id": "990312"}}'
    client.publish(TOPIC_PUBLISH, data2send, qos=2)

# 卸载并回抽耗材
def unloadFilament(filament_num):
    wiringpi.digitalWrite(MOTOR_BACKWARD[filament_num], GPIO.HIGH)
    while True:
        if wiringpi.digitalRead(FILAMENT_DETECTION[filament_num]) != GPIO.HIGH:
            wiringpi.digitalWrite(MOTOR_BACKWARD[filament_num], GPIO.LOW) 

# 装载耗材
def loadFilament(filament_num):
    wiringpi.digitalWrite(MOTOR_FORWARD[filament_num], GPIO.HIGH)
    while True:
        if bambu_flament_detection_flag != False:
            wiringpi.digitalWrite(MOTOR_FORWARD[filament_num], GPIO.LOW)
            break

# 当客户端接收到来自服务器的CONNACK响应时的回调
def on_connect(client, userdata, flags, rc, properties):
    global AMS_status
    if rc == 0:
        print("连接竹子成功")
        # 连接成功后订阅主题
        client.subscribe(TOPIC_SUBSCRIBE)
        AMS_status = status.standby
    else:
        print(f"连接竹子失败，错误代码 {rc}")

# 当客户端断开连接时的回调
def on_disconnect(client, userdata, rc, properties):
    global AMS_status
    print("连接已断开，请检查打印机状态，以及是否有其它应用占用了打印机")
    AMS_status = status.bambuOffline

# 当收到服务器发来的消息时的回调
def on_message(client, userdata, message):
    global AMS_status, now_filament, next_filament
    if DEBUG:
        print(f"Received message '{str(message.payload.decode('utf-8'))}' on topic '{message.topic}'")
    try:
        # 尝试解析JSON数据
        payload = str(message.payload.decode('utf-8'))
        json_data = json.loads(payload)
        if DEBUG:
            print(json_data)
        # 这里可以根据需要进一步处理json_data
    except json.JSONDecodeError:
        # 如果消息不是JSON格式，打印错误
        print("JSON解析失败")
        return
    if "print" in json_data:
        if AMS_status == status.standby:
            if "gcode_state" in json_data["print"]:
                if json_data["print"]["gcode_state"] == "PAUSE": # 暂停状态
                    if "mc_percent" in json_data["print"] and "mc_remaining_time" in json_data["print"]:
                        if json_data["print"]["mc_percent"] == 101: # 换色指令
                            print("开始换色")
                            next_filament = json_data["print"]["mc_remaining_time"] # 更换通道
                            if next_filament == now_filament: # 无需更换
                                print("无需更换")
                                client.publish(TOPIC_PUBLISH, bambu_resume, qos=2) # 继续打印
                                return
                            client.publish(TOPIC_PUBLISH, bambu_unload, qos=2) # 卸载耗材丝
                            print("等待卸载完成")
                            AMS_status = status.unloadingFilament
                            unloadFilament(now_filament)
        elif AMS_status == status.unloadingFilament:
            if "hw_switch_state" in json_data["print"]:
                if json_data["hw_switch_state"] == 0: # 断料检测为无料
                    print("卸载完成")
                    now_filament = -1
                    AMS_status = status.AMSloadingFilament
                    loadFilament(next_filament)
        elif AMS_status == status.AMSloadingFilament:
            if "hw_switch_state" in json_data["print"]:
                if json_data["hw_switch_state"] == 1: # 断料检测为有料
                    print("料线到达，开始装载")
                    now_filament = next_filament
                    client.publish(TOPIC_PUBLISH, bambu_load, qos=2)
                    AMS_status = status.bambuLoadingFilament
        elif AMS_status == status.AMSloadingFilament:
            if "ams" in json_data[print]:
                if "tray_tar" in json_data["print"]["ams"]:
                    time.sleep(8) # 等待装载完成 TODO:这里可以通过“print_error: 318734343”来判断转载完成，先用延时省事
                    while True:
                        if BambuFilamentReady():
                            break
                    print("装载完成")
                    client.publish(TOPIC_PUBLISH, bambu_done, qos=2)
                    sendCaliData(client,next_filament)
                    AMS_status = status.bambuLoadingFilament
        elif AMS_status == status.bambuLoadingFilament:
            if "ams" in json_data[print]:
                if "tray_pre" in json_data["print"]["ams"]:
                    if json_data["print"]["ams"]["tray_pre"] == 254:
                        print("冲刷完成，继续打印")
                        client.publish(TOPIC_PUBLISH, bambu_resume, qos=2)
                        BambuFilamentReady = status.standby