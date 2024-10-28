#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "message_center.h"
#include "ins_task.h"
#include "arm_math.h"
#include "distance.h"
#include "MImotor.h"
#include "math.h"
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
static Distance_data *dt_data;
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_x, chassis_y, chassis_z; // 底盘的位置和角度
static float chassis_vx, chassis_vy ;// 解算后的底盘速度
// static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
// static PIDInstance chassis_angle_pid;                     // 底盘的PID控制器
// static PIDInstance chassis_distance_pid;                      // 底盘的PID控制器
uint8_t send_buf [144]; //
static uint8_t tongdianflag;
static MIMotorInstance *lf0,*lf1,*lf2,*rf0,*rf1,*rf2,*lb0,*lb1,*lb2,*rb0,*rb1,*rb2;
 
void ChassisInit()
{   
    tongdianflag=0;    
    dt_data = dtInit(&huart1); //获得测距传感器数据
        //底盘距离环pid初始化、
    //----------------测试小米电机------------------
    Motor_Init_Config_s mi__motor_config = {
        
        .can_init_config.can_handle = &hcan2,
        .controller_param_init_config = {
            .angle_PID={
                .Kp=1,//运控的kp
            },
            .speed_PID = {
                .Kp = 10, // 
                .Ki = 0,  // 0
                .Kd = 0,  //运控的kd
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 25,//速度限制
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 23,//电流限制
            },
        },
        .controller_setting_init_config = {
            .ifsetflag=0,  
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type =ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP, 
        },
        .motor_type =  MI,
    };
    mi__motor_config.can_init_config.tx_id =11;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag = 0;
    lf0 = MIMotorInit(&mi__motor_config);
    
    // mi__motor_config.controller_setting_init_config.close_loop_type=SPEED_LOOP;
    mi__motor_config.can_init_config.tx_id = 12;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag =1;
    lf1 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 13;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag = 0;
    lf2 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 21;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag = 1;
    rf0 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 22;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag =0;
    rf1 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 23;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag =1;
    rf2 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.can_handle = &hcan1 ;
        
    mi__motor_config.can_init_config.tx_id = 31;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag =1;
    lb0 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 32;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag =0;
    lb1 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 33;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag =1;
    lb2 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 41;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag = 0;
    rb0 = MIMotorInit(&mi__motor_config);

    mi__motor_config.can_init_config.tx_id = 42;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag = 1;
    rb1 = MIMotorInit(&mi__motor_config);
   
    mi__motor_config.can_init_config.tx_id = 43;
    mi__motor_config.can_init_config.tx_ide=CAN_ID_EXT;
    mi__motor_config.controller_setting_init_config.motor_reverse_flag = 0;
    rb2 = MIMotorInit(&mi__motor_config);
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311, 
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)




void send_motor_measurements(UART_HandleTypeDef *huart, uint8_t *send_buf, MIMotorInstance *motor, uint8_t motor_id) {
    int offset = 0;
    send_buf[offset++] = 'A';
    send_buf[offset++] = motor_id;


    memcpy(send_buf + offset, &motor->measure.angle_single_round, 4);
    offset += 4;
    memcpy(send_buf + offset, &motor->measure.speed_aps, 4);
    offset += 4;
    memcpy(send_buf + offset, &motor->measure.real_current, 4);
    offset += 4;
    
    send_buf[offset++] = 'S';

    HAL_UART_Transmit_DMA(huart, send_buf, 15);
}


/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
 void MecanumCalculate()
{    
    
    uint8_t send_buf[15];
    send_motor_measurements(&huart1, send_buf, lf0,1); // 假设lf0的ID是0x0001
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, lf1,2); // 假设lf1的ID是0x0002
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, lf2, 3); // 假设lf2的ID是0x0003
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, rf0, 4); // 假设rf0的ID是0x0004
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, rf1, 5); // 假设rf1的ID是0x0005
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, rf2, 6); // 假设rf2的ID是0x0006
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, lb0,7); // 假设lb0的ID是0x0007
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, lb1, 8); // 假设lb1的ID是0x0008
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, lb2,9); // 假设lb2的ID是0x0009
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, rb0, 10); // 假设rb0的ID是0x000A
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, rb1,11); // 假设rb1的ID是0x000B
    osDelay(1);
    send_motor_measurements(&huart1, send_buf, rb2, 12); // 假设rb2的ID是0x000C
    osDelay(1);
}    

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值  
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    // MIMotorSetRef(lf0, 0 );
    // MIMotorSetRef(lf1,0);
    // MIMotorSetRef(motor_rf, vt_rf);
    // DJIMotorSetRef(motor_lb, vt_lb);
    // DJIMotorSetRef(motor_rb, vt_rb);
}


/**
 * @brief 对控制的底盘距离进行解算
 *          
 *
 */

// static void EstimateChassisDistance(PIDInstance *chassis_distance_pid,float distance_measure,float distance_ref){
//         chassis_cmd_recv.vy= -1*PIDCalculate(chassis_distance_pid, distance_measure, distance_ref);
// }


/**
 * @brief 对控制的底盘角度进行解算
 *          
 *
 */

// static void EstimateChassisAngle(PIDInstance *chassis_angle_pid,float angle_measure,float angle_ref){
//         chassis_cmd_recv.wz= -1*PIDCalculate(chassis_angle_pid, angle_measure, angle_ref);
// }

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed(){
   if(dt_data->type==0){
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
        tongdianflag==0;
   }
   if(dt_data->type==1){
        tongdianflag=1;
        MIMotorOuterLoop(lf0,ANGLE_LOOP);
        MIMotorOuterLoop(lf1,ANGLE_LOOP);
        MIMotorOuterLoop(lf2,ANGLE_LOOP);
         MIMotorOuterLoop(rf0,ANGLE_LOOP);
        MIMotorOuterLoop(rf1,ANGLE_LOOP);
        MIMotorOuterLoop(rf2,ANGLE_LOOP);
         MIMotorOuterLoop(lb0,ANGLE_LOOP);
        MIMotorOuterLoop(lb1,ANGLE_LOOP);
        MIMotorOuterLoop(lb2,ANGLE_LOOP);
         MIMotorOuterLoop(rb0,ANGLE_LOOP);
        MIMotorOuterLoop(rb1,ANGLE_LOOP);
        MIMotorOuterLoop(rb2,ANGLE_LOOP);
        MIMotorSetRef(lf0,dt_data->type1.ref1*(PI/180));
        MIMotorSetRef(lf1, dt_data->type1.ref2*(PI/180));
        MIMotorSetRef(lf2, dt_data->type1.ref3*(PI/180) );
        MIMotorSetRef(rf0, dt_data->type1.ref4*(PI/180) );
        MIMotorSetRef(rf1, dt_data->type1.ref5*(PI/180));
        MIMotorSetRef(rf2, dt_data->type1.ref6*(PI/180) );
        MIMotorSetRef(lb0, dt_data->type1.ref7*(PI/180) );
        MIMotorSetRef(lb1, dt_data->type1.ref8 *(PI/180));
        MIMotorSetRef(lb2, dt_data->type1.ref9 *(PI/180));
        MIMotorSetRef(rb0, dt_data->type1.ref10 *(PI/180));
        MIMotorSetRef(rb1, dt_data->type1.ref11*(PI/180) );
        MIMotorSetRef(rb2,dt_data->type1.ref12*(PI/180));

   }
   if(dt_data->type==2){
        tongdianflag=1;
        if(dt_data->type2.id1==11){
            MIMotorOuterLoop(lf0,ZERO_LOC);
            MIMotorSetRef(lf0,0);
        } 
        if(dt_data->type2.id1==12){
            MIMotorOuterLoop(lf1,ZERO_LOC);
            MIMotorSetRef(lf1,0);
        }
        if(dt_data->type2.id1==13){
            MIMotorOuterLoop(lf2,ZERO_LOC);
            MIMotorSetRef(lf2,0);
        }
        if(dt_data->type2.id1==21){
            MIMotorOuterLoop(rf0,ZERO_LOC);
            MIMotorSetRef(rf0,0);
        }
        if(dt_data->type2.id1==22){
            MIMotorOuterLoop(rf1,ZERO_LOC);
            MIMotorSetRef(rf1,0);
        }
        if(dt_data->type2.id1==23){
            MIMotorOuterLoop(rf2,ZERO_LOC);
            MIMotorSetRef(rf2,0);
        }
        if(dt_data->type2.id1==31){
            MIMotorOuterLoop(lb0,ZERO_LOC);
            MIMotorSetRef(lb0,0);
        }   
        if(dt_data->type2.id1==32){
            MIMotorOuterLoop(lb1,ZERO_LOC);
            MIMotorSetRef(lb1,0);
        }
        if(dt_data->type2.id1==33){
            MIMotorOuterLoop(lb2,ZERO_LOC);
            MIMotorSetRef(lb2,0);
        }
        if(dt_data->type2.id1==41){
            MIMotorOuterLoop(rb0,ZERO_LOC);
            MIMotorSetRef(rb0,0);
        }
        if(dt_data->type2.id1==42){
            MIMotorOuterLoop(rb1,ZERO_LOC);
            MIMotorSetRef(rb1,0);
        }
        if(dt_data->type2.id1==43){
            MIMotorOuterLoop(rb2,ZERO_LOC);
            MIMotorSetRef(rb2,0);
        }


        if(dt_data->type2.id2==11){
            MIMotorOuterLoop(lf0,ZERO_LOC);
            MIMotorSetRef(lf0,0);
        }
        if(dt_data->type2.id2==12){
            MIMotorOuterLoop(lf1,ZERO_LOC);
            MIMotorSetRef(lf1,0);
        }
        if(dt_data->type2.id2==13){
            MIMotorOuterLoop(lf2,ZERO_LOC);
            MIMotorSetRef(lf2,0);
        }
        if(dt_data->type2.id2==21){
            MIMotorOuterLoop(rf0,ZERO_LOC);
            MIMotorSetRef(rf0,0);
        }
        if(dt_data->type2.id2==22){
            MIMotorOuterLoop(rf1,ZERO_LOC);
            MIMotorSetRef(rf1,0);
        }
        if(dt_data->type2.id2==23){
            MIMotorOuterLoop(rf2,ZERO_LOC);
            MIMotorSetRef(rf2,0);
        }
        if(dt_data->type2.id2==31){
            MIMotorOuterLoop(lb0,ZERO_LOC);
            MIMotorSetRef(lb0,0);
        }   
        if(dt_data->type2.id2==32){
            MIMotorOuterLoop(lb1,ZERO_LOC);
            MIMotorSetRef(lb1,0);
        }
        if(dt_data->type2.id2==33){
            MIMotorOuterLoop(lb2,ZERO_LOC);
            MIMotorSetRef(lb2,0);
        }
        if(dt_data->type2.id2==41){
            MIMotorOuterLoop(rb0,ZERO_LOC);
            MIMotorSetRef(rb0,0);
        }
        if(dt_data->type2.id2==42){
            MIMotorOuterLoop(rb1,ZERO_LOC);
            MIMotorSetRef(rb1,0);
        }
        if(dt_data->type2.id2==43){
            MIMotorOuterLoop(rb2,ZERO_LOC);
            MIMotorSetRef(rb2,0);
        }

        if(dt_data->type2.id3==11){
            MIMotorOuterLoop(lf0,ZERO_LOC);
            MIMotorSetRef(lf0,0);
        }
        if(dt_data->type2.id3==12){
            MIMotorOuterLoop(lf1,ZERO_LOC);
            MIMotorSetRef(lf1,0);
        }
        if(dt_data->type2.id3==13){
            MIMotorOuterLoop(lf2,ZERO_LOC);
            MIMotorSetRef(lf2,0);
        }
        if(dt_data->type2.id3==21){
            MIMotorOuterLoop(rf0,ZERO_LOC);
            MIMotorSetRef(rf0,0);
        }
        if(dt_data->type2.id3==22){
            MIMotorOuterLoop(rf1,ZERO_LOC);
            MIMotorSetRef(rf1,0);
        }
        if(dt_data->type2.id3==23){
            MIMotorOuterLoop(rf2,ZERO_LOC);
            MIMotorSetRef(rf2,0);
        }
        if(dt_data->type2.id3==31){
            MIMotorOuterLoop(lb0,ZERO_LOC);
            MIMotorSetRef(lb0,0);
        }   
        if(dt_data->type2.id3==32){
            MIMotorOuterLoop(lb1,ZERO_LOC);
            MIMotorSetRef(lb1,0);
        }
        if(dt_data->type2.id3==33){
            MIMotorOuterLoop(lb2,ZERO_LOC);
            MIMotorSetRef(lb2,0);
        }
        if(dt_data->type2.id3==41){
            MIMotorOuterLoop(rb0,ZERO_LOC);
            MIMotorSetRef(rb0,0);
        }
        if(dt_data->type2.id3==42){
            MIMotorOuterLoop(rb1,ZERO_LOC);
            MIMotorSetRef(rb1,0);
        }
        if(dt_data->type2.id3==43){
            MIMotorOuterLoop(rb2,ZERO_LOC);
            MIMotorSetRef(rb2,0);
        }
   }
   if(dt_data->type==3){
        if(dt_data->type3.id==1){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(lf0,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(lf0,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(lf0,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==2){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(lf1,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(lf1,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(lf1,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==3){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(lf2,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(lf2,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(lf2,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==4){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(rf0,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(rf0,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(rf0,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==5){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(rf1,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(rf1,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(rf1,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==6){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(rf2,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(rf2,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(rf2,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==7){
           if(dt_data->type3.mode==1){
                MIMotorOuterLoop(lb0,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(lb0,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(lb0,CURRENT_LOOP);
            }
        }   
        if(dt_data->type3.id==8){
           if(dt_data->type3.mode==1){
                MIMotorOuterLoop(lb1,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(lb1,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(lb1,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==9){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(lb2,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(lb2,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(lb2,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==10){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(rb0,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(rb0,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(rb0,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==11){
            if(dt_data->type3.mode==1){
                MIMotorOuterLoop(rb1,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(rb1,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(rb1,CURRENT_LOOP);
            }
        }
        if(dt_data->type3.id==12){
             if(dt_data->type3.mode==1){
                MIMotorOuterLoop(rb2,ANGLE_LOOP);
            }else if(dt_data->type3.mode==2){
                MIMotorOuterLoop(rb2,SPEED_LOOP);
            }else if(dt_data->type3.mode==3){
                MIMotorOuterLoop(rb2,CURRENT_LOOP);
            }
        }
        
   }




//    反馈
    // lf0->measure.
}


    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)

    // chassis_feedback_data.vx vy wz =
    //  ...}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif   
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD
    EstimateSpeed();
    // static float angle1=0.1;
    // static int n=0;
    // n=n+1;
    // float time_step = 0.01; // 例如，每步增加0.01秒
    // float amplitude = 0.1; // 正弦波的幅度为0.4
    // float frequency = 0.5; // 正弦波的频率，单位：赫兹
    // angle1 = amplitude * sin(2 * PI * frequency * (n * time_step));

    // if(n%1000==0){
    //     angle1=-1*angle1;
    //     n=0;
    // }
   
    // MIMotorSetRef(lf0,angle1);
    // MIMotorSetRef(lf1,angle1);
    // MIMotorSetRef(lf2,angle1);
    // MIMotorSetRef(rf0,angle1);
    // MIMotorSetRef(rf1,angle1);
    // MIMotorSetRef(rf2,angle1);
    // MIMotorSetRef(rb0,angle1);
    // MIMotorSetRef(rb1,angle1);
    // MIMotorSetRef(rb2,angle1);
    // MIMotorSetRef(lb0,angle1);
    // MIMotorSetRef(lb1,angle1);
    // MIMotorSetRef(lb2,angle1);
    // dt_data->type=1;
    // if(tongdianflag==0){
    //     lf0->tongdianflag=1;
    //     lf1->tongdianflag=1;
    //     lf2->tongdianflag=1;
    //     rf0->tongdianflag=1;
    //     rf1->tongdianflag=1;
    //     rf2->tongdianflag=1;
    //     lb0->tongdianflag=1;
    //     lb1->tongdianflag=1;
    //     lb2->tongdianflag=1;
    //     rb0->tongdianflag=1;
    //     rb1->tongdianflag=1;
    //     rb2->tongdianflag=1;

    // }else{
    //     lf0->tongdianflag=0;
    //     lf1->tongdianflag=0;
    //     lf2->tongdianflag=0;
    //     rf0->tongdianflag=0;
    //     rf1->tongdianflag=0;
    //     rf2->tongdianflag=0;
    //     lb0->tongdianflag=0;
    //     lb1->tongdianflag=0;
    //     lb2->tongdianflag=0;
    //     rb0->tongdianflag=0;
    //     rb1->tongdianflag=0;
    //     rb2->tongdianflag=0;
    // }
    if(dt_data->type==0){ // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        MIMotorStop(lf0);
        MIMotorStop(lf1);
        MIMotorStop(lf2);
        MIMotorStop(rf0);
        MIMotorStop(rf1);
        MIMotorStop(rf2);
        MIMotorStop(lb0);
        MIMotorStop(lb1);
        MIMotorStop(lb2);
        MIMotorStop(rb0);
        MIMotorStop(rb1);
        MIMotorStop(rb2);
    }
    else
    { // 正常工作
        MIMotorEnable(lf0);
        MIMotorEnable(lf1);
        MIMotorEnable(lf2);
        MIMotorEnable(rf0);
        MIMotorEnable(rf1);
        MIMotorEnable(rf2);
        MIMotorEnable(lb0);
        MIMotorEnable(lb1);
        MIMotorEnable(lb2);
        MIMotorEnable(rb0);
        MIMotorEnable(rb1);
        MIMotorEnable(rb2);
    }
    chassis_x=chassis_cmd_recv.x;
    chassis_z=INS.total_angle;

    chassis_vx= chassis_cmd_recv.vy;
    chassis_vy=chassis_cmd_recv.vx;

    

    // LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    //发送数据
    // MecanumCalculate();
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}