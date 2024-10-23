#ifndef __AMC_ROS_HPP__
#define __AMC_ROS_HPP__

#include <array>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"
#include <math.h>
#include "amc_base_msg/bms.h"
#include "amc_base_msg/four_wheel_loader.h"
#include "amc_base_msg/four_wheel_odometer.h"
#include "amc_base_msg/four_wheel_speed.h"
#include "amc_base_msg/car_speed.h"
#include "amc_base_serial/amc_serial.hpp"

// 转换成偏航角速度模式
// LZ-600
// #define CARLENGTH       270
// #define CARWIDTH        194
// #define WHEELTRACK      74

// LZ-PRO
// #define CARLENGTH    300
// #define CARWIDTH     236.5
// #define WHEELTRACK   64

// LZ-STD
#define CARLENGTH    210 
#define CARWIDTH     151
#define WHEELTRACK   63

#define pi          (3.1415926F)
// 速度的最大值
#define SPEED_MAX   1000
// 角度的最大值
#define ANGLE_MAX   20
// 1:选择线速度模式控制 0：选择速度模式控制
#define LinearMode  0



class CarBaseRos
{
private:
    ros::NodeHandle nh;
    ros::Publisher bms_pub;
    ros::Publisher speed_pub;
    ros::Subscriber sub;
    amc_base_msg::bms bms_data;
    amc_base_msg::car_speed speed_data;
public:
    CarBaseRos() {
    // 创建BMS话题
    bms_pub = nh.advertise<amc_base_msg::bms>("/bms", 1000);
    // 创建速度话题
    speed_pub = nh.advertise<amc_base_msg::car_speed>("/car_speed", 1000); 
    // 订阅速度话题
    sub = nh.subscribe<geometry_msgs::Twist>(
            "/cmd_vel", 10,
            std::bind(&CarBaseRos::velocityCallback, this, std::placeholders::_1)
        );
    }
    ~CarBaseRos() {}
    void machineEnable();
    void machineDisable();
    void serialConnect(std::string);
    std::string execCommand(const char*);
    uint8_t getCheckValue(const uint8_t*);
    float AngularToAngle(const int16_t&, const int16_t&);
    void LinearModeToVelMode(int16_t&, float&, const int16_t&, const int16_t&);
    void machineRunControl(const int16_t&, const int16_t&);
    void machineRunControl_SpeedMode(const int16_t&, const int16_t&, const int16_t&);
    void velocityCallback(const geometry_msgs::Twist::ConstPtr&);
    void receivedFeedbackMessage();
    void rxDataProcessing(const uint8_t*, const uint8_t&);
};


serial::Serial serialPort;

// 搜索串口号
std::string CarBaseRos::execCommand(const char* cmd) {
    char buffer[128];
    std::string result;
    FILE* pipe = popen(cmd, "r");
    if (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result = buffer;
    }
    pclose(pipe);
    return result;
}
// 连接串口
void CarBaseRos::serialConnect(std::string serialName) {
    std::string serialCmd = "ls /dev/" + serialName;
    // 获取串口号
    std::string ttyUSB_devices = execCommand(serialCmd.c_str());
    std::string usb_devices = "/dev/ttyUSB0";
    usb_devices[11] = ttyUSB_devices[11];
    // 串口配置, 选择要开启的串口号
    serialPort.setPort(usb_devices);
    // 设置波特率
    serialPort.setBaudrate(115200);
    // 超时等待
    serial::Timeout _time =serial::Timeout::simpleTimeout(2000);
    serialPort.setTimeout(_time);
    // 开启串口
    serialPort.open();
    std::cout << "Connect success!" << std::endl;
}
// 数据校验
uint8_t CarBaseRos::getCheckValue(const uint8_t *data) {
    uint8_t temp 		    = 	0;
    uint16_t length        	=   data[2] + 6U;
    for(int i=2; i<length-1; i++)
        temp ^= data[i];
    return temp;
}

// 发布话题数据
void CarBaseRos::rxDataProcessing(const uint8_t *rx_data, const uint8_t& length) {
    
    std::array<uint8_t, 255U> data;
    for(uint8_t i; i<length; i++) {
        // 查找帧头
        if (rx_data[i] == 0x0A && rx_data[i+1U] == 0x0C) {
            // 填装数据
            if (rx_data[i+2]+6 < sizeof(data))  {
                memmove(data.begin(), &rx_data[i], rx_data[i+2]+6);
            }
            // 数据校验
            if (data.at(2) > 0 && getCheckValue(data.data()) == data.at(data.at(2)+5)) {
                switch (data.at(4)) {
                    // 速度信息
                    case 0x6A:
                        // 时间戳
                        speed_data.stamp = ros::Time::now();
                        // 线速度
                        speed_data.linear = static_cast<int16_t>(data[5] | data[6] << 8) / 1000.0;
                        // 角速度
                        speed_data.angular = static_cast<int16_t>(data[9] | (data[10] << 8)) / 1000.0;
                        // 发布速度话题
                        speed_pub.publish(speed_data);
                        break;
                    // bms信息
                    case 0x69:
                        bms_data.stamp = ros::Time::now();
                        // 剩余电量
                        bms_data.volume_Percentage = static_cast<uint16_t>(data[5] | (data[6] << 8)) / 100.0;
                        // 电流 A
                        bms_data.current_A = static_cast<int16_t>(data[7] | (data[8] << 8)) / 100.0;
                        // 电压 V
                        bms_data.voltage_V = static_cast<int16_t>(data[9] | (data[10] << 8)) / 100.0;
                        bms_pub.publish(bms_data);
                        break;
                }
            }
        }
    }
}

// 使能
void CarBaseRos::machineEnable() {
    UartMessage<0x01U> message;
    message.setCmd(0x01);
    message.setData(0, 0x01);
    serialPort.write(message.check().begin(),message.size());
}

// 失能
void CarBaseRos::machineDisable() {
    UartMessage<0x01U> message;
    message.setCmd(0x01);
    message.setData(0, 0x00);
    serialPort.write(message.check().begin(),message.size());
}

// 角速度转换成角度
float CarBaseRos::AngularToAngle(const int16_t& linear, const int16_t& angular) {
	float R;
	float angle;
	R = (float)linear / (float)angular * 1000;
	if(R < 0 && R >= -CARLENGTH) {
		angle = -ANGLE_MAX;
	} else if(R > 0 && R < CARLENGTH) {
		angle = ANGLE_MAX;
	} else{
		angle = (pi / 2 - acos(CARLENGTH / R)) * 180 / pi;
	}
	return angle;
}

// 线速度模式转换成速度模式
void CarBaseRos::LinearModeToVelMode(int16_t& vel, float& ang, const int16_t& linear, const int16_t& angular) {
    if(linear == 0) {
        vel = 	(sqrt(CARLENGTH*CARLENGTH + CARWIDTH*CARWIDTH) + WHEELTRACK) * angular / 1000 ;//速度
        if(vel > SPEED_MAX) {
            vel = 	SPEED_MAX;
        } else if (vel < -SPEED_MAX) {
            vel = -SPEED_MAX;
        }
    } else {
        vel = linear;//速度
        if(vel > SPEED_MAX) {
            vel = 	SPEED_MAX;
        } else if (vel < -SPEED_MAX) {
            vel = -SPEED_MAX;
        }
        /************限速限角度**********/
        ang	 = AngularToAngle(vel, angular);
        if(ang > ANGLE_MAX) {
            ang = ANGLE_MAX;
        } else if (ang < 	-ANGLE_MAX) {
            ang  = 	-ANGLE_MAX;
        }
    }
}

// 发送线速度角速度控制机器运动 mm/s , 0.001rad/s
void CarBaseRos::machineRunControl(const int16_t& linear, const int16_t& angular) {
    UartMessage<0x04U> message;
    message.setCmd(0x06);
    message.setData<uint16_t>(0, linear);
    message.setData<uint16_t>(2, angular);
    serialPort.write(message.check().begin(),message.size());
}

// 底盘运动控制（偏航角速度模式）
void CarBaseRos::machineRunControl_SpeedMode(const int16_t& vel, const int16_t& ang, const int16_t& round_vel) {
    UartMessage<0x06U> message;
    message.setCmd(0x02);
    message.setData<uint16_t>(0, vel);
    message.setData<uint16_t>(2, ang);
    message.setData<uint16_t>(4, round_vel);
    serialPort.write(message.check().begin(),message.size());
}

// 速度话题处理
void CarBaseRos::velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)  {
    // 获取线速度
    int16_t linear_x = msg->linear.x * 1000;
    // 获取角速度
    int16_t angular_z = msg->angular.z * 1000;

    #if LinearMode
        // 线速度模式控m制
        machineRunControl_LinearMode(linear_x, angular_z);
    #else
        //偏航角模式控制
        int16_t vel;
        float   ang;
        LinearModeToVelMode(vel, ang, linear_x, angular_z);
        // 原地转状态
        if (linear_x == 0 && angular_z != 0) {
            machineRunControl_SpeedMode(0, 0, vel);
        } else { 
            machineRunControl_SpeedMode(vel, ang * 100, 0); // 正常行状态
        } 
    #endif
}

void CarBaseRos::receivedFeedbackMessage() {
    // 存储接收数据
    std::array<uint8_t, 255U> reveiceDataArray;
    if (serialPort.available() > 0) {
        uint8_t length = serialPort.available();
        // 获取串口数据
        serialPort.read(reveiceDataArray.data(),length);
        // 串口数据处理
        rxDataProcessing(reveiceDataArray.data(), length);
        // 清空缓存
        serialPort.flushInput();
    }
}


#endif // !__AMC_ROS_HPP__
