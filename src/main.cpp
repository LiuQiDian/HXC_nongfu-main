/*
 * 
 * 　　┏┓　　　┏┓+ +
 * 　┏┛┻━━━┛┻┓ + +
 * 　┃　　　　　　　┃ 　
 * 　┃　　　━　　　┃ ++ + + +
 *  ████━████ ┃+
 * 　┃　　　　　　　┃ +
 * 　┃　　　┻　　　┃
 * 　┃　　　　　　　┃ + +
 * 　┗━┓　　　┏━┛
 * 　　　┃　　　┃　　　　　　　　　　　
 * 　　　┃　　　┃ + + + +
 * 　　　┃　　　┃
 * 　　　┃　　　┃ +  神兽保佑
 * 　　　┃　　　┃    代码无bug　　
 * 　　　┃　　　┃　　+　　　　　　　　　
 * 　　　┃　 　　┗━━━┓ + +
 * 　　　┃ 　　　　　　　┣┓
 * 　　　┃ 　　　　　　　┏┛
 * 　　　┗┓┓┏━┳┓┏┛ + + + +
 * 　　　　┃┫┫　┃┫┫
 * 　　　　┗┻┛　┗┻┛+ + + +
 * 
 */




#include <Arduino.h>
#include "C600.hpp"
#include "espnow.hpp"
#include "car.hpp"
#include "HEServo.hpp"
HardwareSerial servo_ser(2);//舵机串口
M3508_P19 RFmoto(1);//右前电机
M3508_P19 RBmoto(2);//右后电机
M3508_P19 LBmoto(3);//左后电机
M3508_P19 LFmoto(4);//左前电机
M3508_P19 lift_motor(7);//升降电机
M3508_P19 back_rotate_motor(5);//后夹爪旋转电机
HEServo grap1(&servo_ser,1);//秧苗夹爪
M2006_P36 back_grap_motor(6);//后夹爪
//农夫对象
McNampCar nongfu(&LFmoto,&LBmoto,&RFmoto,&RBmoto,286.1,394.0,152.4);
// HEServo grap1(&servo_ser,1);
float servo_lose_angel=20;
float servo_grap_angle=115;

//控制舵机夹爪的任务
void servo_task (void* n){
  while(1){
    if(grap1.SERVO_TEMP_READ()>70){
      
      grap1.SERVO_LOAD_OR_UNLOAD_WRITE(0);
    }
    else{
      int time=1000;
      if(value.but_state&(1<<6)){
        grap1.SERVO_MOVE_TIME_WRITE(servo_grap_angle,time);
        delay(2*time);
      }else{
        grap1.SERVO_MOVE_TIME_WRITE(servo_lose_angel,time);
        delay(2*time);
      }
    }
    delay(100);
  }
};

//后夹爪初始化，用于获得零点
void grap_setup(){
  back_grap_motor.set_location_pid(0.1,0,1,20000,6000);
  back_grap_motor.set_speed_pid(5,0,0,0,6000);
  back_grap_motor.set_speed(-100);
  delay(300);
  while(back_grap_motor.get_now_speed()!=0){
    Serial.println(back_grap_motor.get_now_speed());
    delay(100);
  }
  back_grap_motor.unload();
  back_grap_motor.reset_location(0);
  back_grap_motor.load();
  delay(500);
}


//后夹爪控制
void grap(bool state){
  if(state){
    back_grap_motor.set_location(130000);
  }else{
    back_grap_motor.set_location(5000);
  }
}

//后夹爪旋转初始化
void grap_rotate_setup(){
  back_rotate_motor.set_location_pid(0.01,0,0,8000,6000);
  back_rotate_motor.set_speed_pid(3,0.01,1,1000,6000);
  back_rotate_motor.set_speed(-50);
  delay(300);
  while(back_rotate_motor.get_now_speed()!=0){
    Serial.println(back_rotate_motor.get_now_speed());
    delay(100);
  }
  back_rotate_motor.unload();
  back_rotate_motor.reset_location(0);
  back_rotate_motor.load();
  delay(500);
}
//后夹爪旋转控制
void grap_rotate(bool state){
  if(state){
    back_rotate_motor.set_location(70000);
  }else{
    back_rotate_motor.set_location(2000);
  }
}

void setup() {
  
  //调试串口初始化
  Serial.begin(115200);

  //舵机串口初始化
  servo_ser.begin(115200,SERIAL_8N1,20,19);

  //can初始化
  can_init(48,47,100);

  /*↓ESPNOW初始化↓*/
  //设置WiFi模式为WIFI_STA（Station模式）
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
      Serial.println("ESP-NOW initialization failed");
      return;
  }
  esp_now_peer_info_t peerInfo;
  peerInfo.ifidx = WIFI_IF_STA;
  memcpy(peerInfo.peer_addr, receive_MACAddress, 6);

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  /*↑ESPNOW初始化↑*/


  delay(3000);
  lift_motor.setup();
  grap_setup();
  grap(true);
  grap_rotate_setup();
  
  //农夫初始化
  nongfu.setup();
  //创建舵机控制任务
  xTaskCreate(servo_task,"servo_task",1024,NULL,2,NULL);

    
}

void loop() {
  float lift_speed=0;
  if(value.but_state&(1<<5)){//停止开关
      //农夫停止
      nongfu.stop();
      //lift_motor.set_speed(0);
  }else{
      //农夫移动
      nongfu.set_speed(vec3<float>(3*value.lx*value.lknob,3*value.ly*value.lknob,3*value.rx*value.rknob));
      lift_speed=75*bool(value.but_state&(1<<3));
      lift_speed -= 75*bool(value.but_state&(1<<2));
      lift_motor.set_speed(lift_speed);
  }
  if(value.but_state&(1<<4)){
    grap(true);
  }else{
    grap(false);
  }
  if(value.but_state&(1<<7)){
    grap_rotate(true);
  }else{
    grap_rotate(false);
  }
  //Serial.println(nongfu.get_position().Y);
  delay(20);
}