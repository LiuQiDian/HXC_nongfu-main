/*
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-04-13 21:00:21
 */
#ifndef espnow_hpp
#define espnow_hpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
uint8_t receive_MACAddress[] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
// 定义一个名为DATA的类，用于接收数据，注意要与发送端相同
class DATA {
public:
    float ly=0;//左摇杆Y轴
    float lx=0;//左摇杆X轴
    float ry=0;//右摇杆Y轴
    float rx=0;//右摇杆X轴
    float lknob=0;//左旋钮
    float rknob=0;//右旋钮
    uint8_t but_state=0;//按键，每一位对应一个按键
};
DATA value;

int64_t last_time_receive_time=0;
//接收数据时的回调函数，收到数据时自动运行
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(value)) {
    //Serial.println("Invalid data packet");
    return;
  }
  memcpy(&value, data, sizeof(value));
  last_time_receive_time=millis();
}



#endif 