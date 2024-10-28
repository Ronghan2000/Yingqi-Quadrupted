#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"
#define MI_MOTOR_CNT 12

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f

#define IQ_REF_MIN -27.0f
#define IQ_REF_MAX 27.0f
#define SPD_REF_MIN -30.0f
#define SPD_REF_MAX 30.0f
#define LIMIT_TORQUE_MIN 0.0f
#define LIMIT_TORQUE_MAX 12.0f
#define CUR_FILT_GAIN_MIN 0.0f
#define CUR_FILT_GAIN_MAX 1.0f
#define LIMIT_SPD_MIN 0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_CUR_MIN 0.0f
#define LIMIT_CUR_MAX 27.0f

typedef struct
{
    // uint16_t last_ecd;        // 上一次读取的编码器值
    // uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    float real_current;     // 实际电流
    float temperature;      // 温度 Celsius
    float motor_id;
    float index;
    float param;
    // float total_angle;   // 总角度,注意方向
    // int32_t total_round; // 总圈数,注意方向
} MI_Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    MI_Motor_Measure_s measure;            // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器
    CANInstance *motor_can_instance; // 电机CAN实例
    // 分组发送设置
    // uint8_t sender_group;
    // uint8_t message_num;
    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance* daemon;
    uint32_t feed_cnt;
    uint32_t tongdianflag;
    float dt;
    uint8_t initflag;
} MIMotorInstance;


typedef struct
{
    uint32_t info : 24;
    uint32_t communication_type : 5;
    uint32_t res : 3;
}__attribute__((packed))  RxCAN_info_s;// 解码内容缓存

typedef struct
{
    uint32_t FE : 8;
    uint32_t motor_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint32_t MCU_id;
}__attribute__((packed))  RxCAN_info_type_0_s;// 通信类型0解码内容

typedef struct
{
    uint32_t master_can_id : 8;
    uint32_t motor_id : 8;
    uint32_t under_voltage_fault : 1;
    uint32_t over_current_fault : 1;
    uint32_t over_temperature_fault : 1;
    uint32_t magnetic_encoding_fault : 1;
    uint32_t HALL_encoding_failure : 1;
    uint32_t unmarked : 1;
    uint32_t mode_state : 2;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    float angle;//(rad)
    float speed;//(rad/s)
    float torque;//(N*m)
    float temperature;//(℃)
}__attribute__((packed))  RxCAN_info_type_2_s; // 通信类型2解码内容

typedef struct
{
    uint32_t motor_id : 8;
    uint32_t master_can_id : 16;
    uint32_t communication_type : 5;
    uint32_t res : 3;
    uint16_t index;
    float param;
}__attribute__((packed))  RxCAN_info_type_17_s;// 通信类型17解码内容
MIMotorInstance *MIMotorInit(Motor_Init_Config_s *config);
void  MI_motor_TorqueControl(MIMotorInstance* hmotor);
void  MI_motor_LocationControl(MIMotorInstance* hmotor);
void  MI_motor_SpeedControl(MIMotorInstance* hmotor);
 void MI_motor_Modeset(MIMotorInstance* hmotor,float param,uint16_t index);
void MIMotorStop(MIMotorInstance *motor);
void MIMotorEnable(MIMotorInstance *motor);
void MIMotorSetRef(MIMotorInstance *motor, float ref);
void  MI_motor_stop(MIMotorInstance* hmotor);
void MIMotorOuterLoop(MIMotorInstance *motor, Closeloop_Type_e outer_loop);