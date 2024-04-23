/*
 * @Description: 用于控制大疆电机
 * @Author: qingmeijiupiao
 * @Date: 2024-04-13 21:00:21
 * @LastEditTime: 2024-04-20 18:53:56
 * @LastEditors: Please set LastEditors
 * @rely:PID_CONTROL.
*/

#ifndef _C600_HPP_
#define _C600_HPP_

#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "PID_CONTROL.hpp"


/*↓↓↓本文件的类和函数↓↓↓*/

//由于接受电调数据的类，用户无需创建对象
class C600_DATA;

//电机基类,电机无减速箱时使用
class MOTOR;

//3508电机类
class M3508_P19;

//2006电机类
class M2006_P36;

//CAN初始化函数，要使用电机，必须先调用此函数初始化CAN通信
void can_init(uint8_t TX_PIN,uint8_t RX_PIN,/*电流更新频率=*/int current_update_hz=100);

//位置闭环控制任务函数,使用位置控制函数时创建此任务,基于速度闭环控制任务函数,每个电机都有自己的位置闭环控制任务，用户无需调用
void location_contral_task(void* n);

//速度闭环控制任务函数，每个电机都有自己的速度闭环控制任务，用户无需调用
void speed_contral_task(void* n);

//总线数据接收和发送线程，全局只有一个任务，用户无需调用
void feedback_update_task(void* n);

//更新电流任务，全局只有一个任务，用户无需调用
void update_current_task(void* p);

/*↑↑↑本文件的类和函数↑↑↑*/


//电调接收数据相关,用户无需创建对象
class C600_DATA{
    //定义为友元，方便调用
    friend void feedback_update_task(void* n);//总线数据接收和发送线程，全局只有一个任务
    friend void update_current_task(void* n); //更新电流任务，全局只有一个任务
    friend void location_contral_task(void* n);
    friend void speed_contral_task(void* n);//速度闭环控制任务线程，每个电机都有自己的速度闭环控制任务，并且与位置闭环控制任务不同时存在
    friend MOTOR;
    friend M3508_P19;
    friend M2006_P36;

public:
    C600_DATA(){}
    ~C600_DATA(){}
    //返回角度，范围：0-360，单位：度
    float get_angle(){
        return angle/8192.0;    
    }
    //返回转子速度，单位：RPM
    int get_speed(){
        return speed;
    }
    //返回电流，单位：mA
    int get_current(){
        return 2e4*current/16384.0;
    }
    //获取电机温度，单位：摄氏度
    int get_tempertrue(){
        return tempertrue;
    }
    //获取多圈位置,每8192为转子1圈
    int64_t get_location(){
        return location;
    }
    //重置当前多圈位置
    void reset_location(int l=0){
        location = l;
    }
    //判断电机是否在线
    bool is_online(){
        //超过100ms没有更新，就认为电机不在线
        return micros()-last_location_update_time<100000;//100ms
    }
//protected:
    //多圈位置获取
    void update_location(){
        int16_t now_angle=angle;
        if (last_location_update_time==0){
            last_location_update_time=micros();
        }

        int now = micros();

        //数据更新频率低或者速度过高时防止丢步
        //double speed_location=speed*(now-last_location_update_time)*0.000001/60;
        //location += int(speed_location)*8192;


        int delta=0;
        //if (abs(speed)<50){
        //判断旋转方向
            if((now_angle+8192-last_angle)%8192<4096){//正转
                delta=now_angle-last_angle;
                if (delta<0){
                    delta+=8192;
                }
            }else{
                delta=now_angle-last_angle;
                if (delta>0){
                    delta-=8192;
                }
            }
        //}
        // if (speed>50){
        //     delta=now_angle-last_angle;
        //     if (delta<0){
        //         delta+=8192;
        //     }
        // }
        // if (speed<-50){
        //     delta=now_angle-last_angle;
        //     if (delta>0){
        //         delta-=8192;
        //     }
        // }
        location += delta;
        last_location_update_time=now;
        last_angle=now_angle;   
    }
    //将CAN数据更新到变量
    void update_data(twai_message_t can_message){
        angle = can_message.data[0]<<8 | can_message.data[1];
        speed = can_message.data[2]<<8 | can_message.data[3];
        current = can_message.data[4]<<8 | can_message.data[5];
        tempertrue = can_message.data[6];
        update_location();
    }
    uint16_t angle=0;
    int16_t speed=0;
    int16_t current=0;
    uint8_t tempertrue=0;
    int16_t set_current=0;
    int64_t location =0;
    bool enable=false;
    int64_t last_location_update_time=0;
    uint16_t last_angle=0;

};


//1-8号电机数据接收对象,用户无需访问
C600_DATA motor_201,motor_202,motor_203,motor_204,motor_205,motor_206,motor_207,motor_208;
C600_DATA* motors[]={&motor_201,&motor_202,&motor_203,&motor_204,&motor_205,&motor_206,&motor_207,&motor_208};








//电机基类，没有减速箱的电机也可以使用这个类
class MOTOR{
        friend void location_contral_task(void* n);
        friend void speed_contral_task(void* n);
        friend void update_current_task(void* n);
    public:
        //id从1-8
        MOTOR(int id){
            //ID超过范围
            if(id<1 || id>8){
                return;
            }
            data = motors[id-1];
            data->enable=true;
            ////设置默认PID参数
            location_pid_contraler.setPram(default_location_pid_parmater);
            speed_pid_contraler.setPram(default_speed_pid_parmater);
        }
        MOTOR(int id,pid_param location_pid,pid_param speed_pid){
            //ID超过范围
            if(id<1 || id>8){
                return;
            }
            data = motors[id-1];
            data->enable=true;
            //设置PID参数
            location_pid_contraler.setPram(location_pid);
            speed_pid_contraler.setPram(speed_pid);
        }
        //初始化,位置闭环,使能
        void setup(){
            data->enable=true;
            if(speed_func_handle==NULL){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&location_func_handle);
            }
        };
        //判断电机是否在线
        bool is_online(){
            return data->is_online();
        }
        //停止电机 need_unload:是否卸载使能
        void stop(bool need_unload=true){
            if(need_unload){
                unload();
            }
            location_taget=data->get_location();
            speed_location_taget=data->get_location();
            taget_speed=0;
            data->set_current=0;
        }
        //设置位置闭环控制参数
        void set_location_pid(float _location_Kp=0,float _location_Ki=0,float _location_Kd=0,float __dead_zone=0,float _max_speed=0){
                location_pid_contraler.Kp=_location_Kp;
                location_pid_contraler.Ki=_location_Ki;
                location_pid_contraler.Kd=_location_Kd;
                location_pid_contraler._dead_zone=__dead_zone;
                location_pid_contraler._max_value=_max_speed;
        }
        //设置位置闭环控制参数
        void set_location_pid(pid_param pid){
            set_location_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
        }
        //设置速度闭环控制参数
        void set_speed_pid(float _speed_Kp=0,float _speed_Ki=0,float _speed_Kd=0,float __dead_zone=0,float _max_curunt=0){ 
                speed_pid_contraler.Kp=_speed_Kp;
                speed_pid_contraler.Ki=_speed_Ki;
                speed_pid_contraler.Kd=_speed_Kd;
                speed_pid_contraler._dead_zone=__dead_zone;
                max_curunt=_max_curunt;
                if(max_curunt>16384){//最大电流限制
                    max_curunt=16384;
                }
                speed_pid_contraler._max_value=_max_curunt;
        }
        //设置速度闭环控制参数
        void set_speed_pid(pid_param pid){
            set_speed_pid(pid.Kp,pid.Ki,pid.Kd,pid._dead_zone,pid._max_value);
        }
        //设置多圈目标位置
        void set_location(int64_t _location){
            //开启位置闭环控制任务
            if(location_func_handle==NULL){
                xTaskCreate(location_contral_task,"location_contral_task",4096,this,2,&location_func_handle);
            }
            location_taget=_location;
        }
        //重置当前多圈位置
        void reset_location(int64_t _location=0){
            data->reset_location(_location);
        }
        //获取当前多圈位置
        int64_t get_location(){
            return data->location;
        }
        //卸载使能
        void unload(){
            if(speed_func_handle!=NULL){
                vTaskDelete(speed_func_handle);
                speed_func_handle=NULL;
            }
            if(location_func_handle!=NULL){
                vTaskDelete(location_func_handle);
                location_func_handle=NULL;
            }
            this->taget_speed=0;
            this->data->set_current=0;
            delay(30);//等待电流更新到电调
            this->data->enable=false;
        }
        //使能
        void load(){
            taget_speed = 0;
            this->data->enable=true;
            if(speed_func_handle==NULL){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&speed_func_handle);
            }
        }
        //获取是否使能
        bool get_is_load(){
            return speed_func_handle!=NULL;
        }
        //获取当前速度
        virtual float get_now_speed(){
            return data->speed;
        }
        //设置目标速度
        virtual void set_speed(float speed){
        }
        float get_reduction_ratio(){
            return reduction_ratio;
        }
    //protected:
        int64_t location_taget=0;//位置环多圈目标位置
        int64_t speed_location_taget=0;//速度环多圈目标位置
        pid_param default_location_pid_parmater={10,0,0,2000,3000};//位置闭环默认控制参数   
        PID_CONTROL location_pid_contraler;//位置闭环控制PID计算对象
        pid_param default_speed_pid_parmater={5,3,0.01,1,10000};//速度闭环默认控制参数
        PID_CONTROL speed_pid_contraler;//速度闭环控制PID计算对象
        float max_curunt=10000;//最大电流值0-16384
        C600_DATA* data;//数据对象
        float taget_speed = 0;//单位RPM
        TaskHandle_t location_func_handle = NULL;//位置闭环控制任务句柄
        TaskHandle_t speed_func_handle = NULL;//速度闭环控制任务句柄
        float reduction_ratio=1;//减速比
};








//3508类
class M3508_P19:public MOTOR{
    public:
        M3508_P19(int id):MOTOR(id){
            reduction_ratio=19.0;
        };
        M3508_P19(int id,pid_param location_pid,pid_param speed_pid):MOTOR(id,location_pid,speed_pid){
            reduction_ratio=19.0;
        };
        //设置减速箱输出速度，单位RPM
        void set_speed(float speed){
            this->data->enable=true;
            if(location_func_handle!=NULL){
                vTaskDelete(location_func_handle);
                location_func_handle=NULL;
            }
            if(speed_func_handle==NULL){
                xTaskCreate(speed_contral_task,"speed_cspeed_func_handleontral_task",4096,this,2,&speed_func_handle);
            }
            taget_speed = speed*19.0;
        }
        //获取减速箱输出速度，单位RPM
        float get_now_speed(){
            return data->speed/19.0;
        }
};





//2006电机类
class M2006_P36:public MOTOR{
    public:
        M2006_P36(int id):MOTOR(id){
            reduction_ratio=36.0;
        };
        M2006_P36(int id,pid_param location_pid,pid_param speed_pid):MOTOR(id,location_pid,speed_pid){
            reduction_ratio=36.0;
        };
        //设置减速箱输出速度，单位RPM
        void set_speed(float speed){
            this->data->enable=true;
            if(location_func_handle!=NULL){
                vTaskDelete(location_func_handle);
                location_func_handle=NULL;
            }
            if(speed_func_handle==NULL){
                xTaskCreate(speed_contral_task,"speed_contral_task",4096,this,2,&speed_func_handle);
            }
            taget_speed = speed*36.0;
        }
        //获取减速箱输出速度，单位RPM
        float get_now_speed(){
            return data->speed/36.0;
        }
};




//位置闭环控制任务;
void location_contral_task(void* n){
    MOTOR* moto = (MOTOR*) n;
    moto->location_pid_contraler.reset();//重置位置闭环控制器
    float speed=0;
    while (1){
        //位置闭环控制,由位置误差决定速度,再由速度误差决定电流
        speed = moto->location_pid_contraler.control(moto->location_taget - moto->data->location);
        moto->taget_speed = speed;
        delay(10);
    }
};




//速度闭环控制任务
void speed_contral_task(void* n){
    MOTOR* moto = (MOTOR*) n;
    //上次更新时间
    int last_update_speed_time=micros();
    moto->speed_pid_contraler.reset();
    //防止在unload过后情况下，出现扰动，再次load之后不会回到扰动前的位置
    moto->speed_location_taget = moto->data->location;
    int16_t cru;
    while (1){
        //由速度计算得到的目标位置
        moto->speed_location_taget+=8192.0*moto->taget_speed*float(1e-6*(micros()-last_update_speed_time))/60;//位置误差
        //更新上次更新时间
        last_update_speed_time=micros();


        //由速度误差和位置误差一同计算电流
        double err = 
        /*速度环的误差=*/(moto->taget_speed - moto->data->speed)
        +/*速度环位置误差比例系数=*/400* /*这里的比例系数需要根据实际情况调整,比例系数400可以理解为转子每相差一圈加400RPM速度补偿*/
        /*由速度计算得到的目标位置的误差*/(moto->speed_location_taget-moto->data->location)/8192;

        /*PID控制器的计算电流*/
        int16_t cru = moto->speed_pid_contraler.control(err);
        moto->data->set_current = cru;//设置电流
        delay(10);//100HZ更新频率
    }
}




//接收CAN总线上的数据的任务函数
void feedback_update_task(void* n){
    twai_message_t rx_message;
    while (1){
        //接收CAN上数据
        ESP_ERROR_CHECK(twai_receive(&rx_message, portMAX_DELAY));
        //如果是电机数据就更新到对应的对象
        if(rx_message.identifier>=0x201 && rx_message.identifier<=0x208){
            motors[rx_message.identifier-0x201]->update_data(rx_message);
        }else if (rx_message.identifier==0x0){
            /*如果can上有其他数据，可以在这里添加*/
        }
        
    }
    
}



//更新电流控制任务
void update_current_task(void* p){
    
    //电流控制频率
    int frc=*(int*) p;

    while(1){
        //如果启用了1-4号任意一个电机就更新电流
        if(motor_201.enable || motor_202.enable || motor_203.enable || motor_204.enable){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x200;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_201.set_current >> 8;
            tx_msg.data[1] = motor_201.set_current&0xff;
            tx_msg.data[2] = motor_202.set_current >> 8;
            tx_msg.data[3] = motor_202.set_current&0xff;
            tx_msg.data[4] = motor_203.set_current >> 8;
            tx_msg.data[5] = motor_203.set_current&0xff;
            tx_msg.data[6] = motor_204.set_current >> 8;
            tx_msg.data[7] = motor_204.set_current&0xff;
            esp_err_t result= twai_transmit(&tx_msg,portMAX_DELAY);
        }
        //如果启用了5-8号任意一个电机就更新电流
        if(motor_205.enable || motor_206.enable || motor_207.enable || motor_208.enable){
            twai_message_t tx_msg;
            tx_msg.data_length_code=8;
            tx_msg.identifier = 0x1FF;
            tx_msg.self=0;
            tx_msg.extd=0;
            tx_msg.data[0] = motor_205.set_current >> 8;
            tx_msg.data[1] = motor_205.set_current&0xff;
            tx_msg.data[2] = motor_206.set_current >> 8;
            tx_msg.data[3] = motor_206.set_current&0xff;
            tx_msg.data[4] = motor_207.set_current >> 8;
            tx_msg.data[5] = motor_207.set_current&0xff;
            tx_msg.data[6] = motor_208.set_current >> 8;
            tx_msg.data[7] = motor_208.set_current&0xff;
            esp_err_t result= twai_transmit(&tx_msg,portMAX_DELAY);
        }
        //延时
        delay(1000/frc);
    }
}






//初始化CAN总线
void can_init(uint8_t TX_PIN, uint8_t RX_PIN,int current_update_hz){
    
    //总线速率,1Mbps
    static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    //滤波器设置,接受所有地址的数据
    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    //总线配置
    static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t(TX_PIN), gpio_num_t(RX_PIN), TWAI_MODE_NO_ACK);

    //传入驱动配置信息
    twai_driver_install(&g_config, &t_config, &f_config);
    
    //CAN驱动启动
    twai_start();
    //创建任务
    
    xTaskCreate(feedback_update_task,"moto_fb",4096,NULL,5,NULL);//电机反馈任务
    xTaskCreate(update_current_task,"update_current_task",4096,&current_update_hz,5,NULL);//电流控制任务
}



#endif