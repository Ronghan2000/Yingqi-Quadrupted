#include "MImotor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"

static uint8_t idx=0 ; 
static MIMotorInstance *mi_motor_instance[MI_MOTOR_CNT] = {NULL}; 
uint8_t MI_MASTERID = 0; //master id 发送指令时EXTID的bit8:15,反馈的bit0:7
/**
  * @brief          小米电机反馈帧解码（通信类型2）
  * @param[in]      Rx_can_info 接受到的电机数据结构体
  * @param[in]      rx_data[8] CAN线接收到的数据
  * @note           将接收到的CAN线数据解码到电机数据结构体中
  * @retval         none
  */
void MI_motor_RxDecode(RxCAN_info_type_2_s* RxCAN_info,uint8_t rx_data[8])
{
    uint16_t decode_temp_mi;//小米电机反馈数据解码缓冲
    decode_temp_mi = (rx_data[0] << 8 | rx_data[1]);
    RxCAN_info->angle = ((float)decode_temp_mi-32767.5)/32767.5*4*3.1415926f;//rad  

    decode_temp_mi = (rx_data[2] << 8 | rx_data[3]);
    RxCAN_info->speed = ((float)decode_temp_mi-32767.5)/32767.5*30.0f;//单位rad/s

    decode_temp_mi = (rx_data[4] << 8 | rx_data[5]);
    RxCAN_info->torque = ((float)decode_temp_mi-32767.5)/32767.5*12.0f;//Nm

    decode_temp_mi = (rx_data[6] << 8 | rx_data[7]);
    RxCAN_info->temperature = (float)decode_temp_mi/10.0f;//摄氏度
 }
static void DecodeMIMotor(CANInstance *_instance)
{
    uint8_t *rxbuff=_instance->rx_buff;
    MIMotorInstance *motor = (MIMotorInstance *)_instance->id;
    MI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销
    RxCAN_info_s  RxCAN_info;//储存接受的标识符
    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    memcpy(&RxCAN_info,&_instance->rxconf.ExtId,4);//将扩展标识符的内容解码到缓存区，获取通信类型
    if(RxCAN_info.communication_type == 0){//通信类型0的反馈帧解码
        RxCAN_info_type_0_s RxCAN_info_type_0;
        memcpy(&RxCAN_info_type_0,&_instance->rxconf.ExtId,4);//将扩展标识符的内容解码成通信类型0的对应内容
        // memcpy(&RxCAN_info_type_0.MCU_id,&_instance->rxconf.ExtId,8);//获取MCU标识符
        measure->motor_id=RxCAN_info_type_0.motor_id;
    }else if(RxCAN_info.communication_type == 2){//通信类型2的反馈帧解码
        RxCAN_info_type_2_s RxCAN_info_type_2;
        memcpy(&RxCAN_info_type_2,&_instance->rxconf.ExtId,4);//将扩展标识符的内容解码成通信类型2的对应内容
        MI_motor_RxDecode(&RxCAN_info_type_2,rxbuff);//通信类型2的数据解码
        measure->angle_single_round=RxCAN_info_type_2.angle;
        measure->real_current=RxCAN_info_type_2.torque;
        measure->speed_aps=RxCAN_info_type_2.speed;
        measure->temperature=RxCAN_info_type_2.temperature;
    }else if(RxCAN_info.communication_type == 17){//通信类型17的反馈帧解码
        RxCAN_info_type_17_s RxCAN_info_type_17;
        memcpy(&RxCAN_info_type_17,&_instance->rxconf.ExtId,4);//将扩展标识符的内容解码成通信类型17的对应内容
        memcpy(&RxCAN_info_type_17.index,&_instance->rx_buff[0],2);//获取查找的参数索引码
        memcpy(&RxCAN_info_type_17.param,&_instance->rx_buff[0],4);//获取查找的参数信息
        measure->motor_id=RxCAN_info_type_17.motor_id;
        measure->index=RxCAN_info_type_17.index;
        measure->param=RxCAN_info_type_17.param;
    }


}

static void MIMotorLostCallback(void *motor_ptr)
{
    MIMotorInstance *motor = (MIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[mi_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
}
MIMotorInstance *MIMotorInit(Motor_Init_Config_s *config)
{
    MIMotorInstance *instance = (MIMotorInstance *)malloc(sizeof(MIMotorInstance));
    memset(instance, 0, sizeof(MIMotorInstance));
    instance->motor_type = config->motor_type;
    instance->motor_settings = config->controller_setting_init_config;
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    config->can_init_config.rx_id=config->can_init_config.tx_id;
    config->can_init_config.can_module_callback=DecodeMIMotor;
    config->can_init_config.id=instance;   
    config->can_init_config.EXT_ID.fields.motor_id=config->can_init_config.tx_id;
    config->can_init_config.EXT_ID.fields.data=0;
    config->can_init_config.EXT_ID.fields.mode=3;
    config->can_init_config.EXT_ID.fields.res=0;
    instance->motor_can_instance = CANRegister(&config->can_init_config);//注册小米电机到can总线
    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = MIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);
    instance->tongdianflag=0;
    MIMotorEnable(instance);
    mi_motor_instance[idx++] = instance;
    return instance;
}

void MIMotorControl()
{
    MIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting;// 电机控制参数
    // Motor_Controller_s *motor_controller;   // 电机控制器
    // MI_Motor_Measure_s *measure;           // 电机测量值
    static float ref;
    for(size_t i=0;i<idx;i++)
    {
        motor = mi_motor_instance[i];
        motor_setting = &motor->motor_settings;
        // motor_controller = &motor->motor_controller;
        // measure = &motor->measure;
        ref = motor->motor_controller.pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if(motor->tongdianflag==0){
            MI_motor_setzero( motor);
            osDelay(20);
            motor->tongdianflag=1;
        }
        if(motor->motor_settings.motor_reverse_flag==0){
            ref=ref*-1;
        }
        if(motor->stop_flag == MOTOR_STOP){
              MI_motor_stop(motor);
              motor_setting->ifsetflag=0;
        }else if(motor->stop_flag == MOTOR_ENALBED&&motor_setting->ifsetflag==0){
            if((motor_setting->close_loop_type==ANGLE_LOOP&&motor_setting->ifsetflag==0)){
                    // osDelay(20);
                    // if(motor->tongdianflag==1){
                    
                    
                    
                    MI_motor_LocationControl(motor);
                    motor_setting->ifsetflag=1;
            }else if(motor_setting->close_loop_type==SPEED_LOOP&&motor_setting->ifsetflag==0)
            {
                    MI_motor_SpeedControl(motor);
                    motor_setting->ifsetflag=1;
            }else if(motor_setting->close_loop_type==CURRENT_LOOP&&motor_setting->ifsetflag==0)
           {
                    MI_motor_TorqueControl(motor);
                    motor_setting->ifsetflag=1;
            }else if(motor_setting->close_loop_type==YUNDONG_LOOP&&motor_setting->ifsetflag==0){
                    MI_motor_YunkongControl(motor);
                    motor_setting->ifsetflag=1;
            }
         }
         if(motor->stop_flag == MOTOR_ENALBED&&motor_setting->ifsetflag==1){
            if((motor_setting->close_loop_type==ANGLE_LOOP)){
                    MI_motor_Modeset(motor,ref,0x7016);
                     osDelay(1);
            }else if(motor_setting->close_loop_type==SPEED_LOOP){
                    MI_motor_Modeset(motor,ref,0x700A);
                     osDelay(5);
            }else if(motor_setting->close_loop_type==CURRENT_LOOP){
                    MI_motor_Modeset(motor,ref,0x7006);
                     osDelay(5);
            }else if(motor_setting->close_loop_type==YUNDONG_LOOP){

            }
         }
         if(motor_setting->close_loop_type==ZERO_LOC){
                MI_motor_setzero( motor);
                motor_setting->ifsetflag==0;
         }
    }
    
}
void MIMotorStop(MIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void MIMotorEnable(MIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void MIMotorOuterLoop(MIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.close_loop_type = outer_loop;
}

// 设置参考值
void MIMotorSetRef(MIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

// 为所有电机实例计算三环PID,发送控制报文

//--------
/**
  * @brief          float转int，数据打包用
  * @param[in]      x float数值
  * @param[in]      x_min float数值的最小值
  * @param[in]      x_max float数值的最大值
  * @param[in]      bits  int的数据位数
  * @retval         none
  */
uint32_t FloatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x=x_min;
    return (uint32_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief          输入范围限制
  * @param[in]      x 输入数值
  * @param[in]      x_min 输入数值的最小值
  * @param[in]      x_max 输入数值的最大值
  * @retval         none
  */
float RangeRestrict(float x, float x_min, float x_max)
{
    float res;
    if(x > x_max) res=x_max;
    else if(x < x_min) res=x_min;
    else res = x;
    return res;
}

//------------------通讯类型魔改小米文档-------------
/**
  * @brief          获取设备ID （通信类型0），需在电机使能前使用
  * @param[in]      hmotor 电机结构体
  * @retval         none
  */
static void MI_motor_GetID(MIMotorInstance  *hmotor)
{   
    static EXT_ID_T EXT  ;
    EXT.fields.mode=0;
    EXT.fields.data=0;
    EXT.fields.motor_id=0;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId= EXT.raw;
    // hmotor->EXT_ID.mode = 0;
    // hmotor->EXT_ID.data = 0;
    // hmotor->EXT_ID.motor_id = 0;
    // hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->motor_can_instance->tx_buff[i]=0;
    }
    CANTransmit(hmotor->motor_can_instance,1);
     
    
}

/**
  * @brief          运控模式电机控制指令（通信类型1）
  * @param[in]      hmotor 电机结构体
  * @param[in]      torque 目标力矩
  * @param[in]      MechPosition 
  * @param[in]      speed 
  * @param[in]      kp 
  * @param[in]      kd 
  * @retval         none
  */
static void MI_motor_Control(MIMotorInstance* hmotor, float torque, float MechPosition , float speed , float kp , float kd)
{  
    static EXT_ID_T EXT  ;
    EXT.fields.mode=1;
    EXT.fields.data=FloatToUint(torque,T_MIN,T_MAX,16);
    EXT.fields.motor_id=hmotor->motor_can_instance->tx_id;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId=EXT.raw;
    // hmotor->EXT_ID.mode = 1;
    // hmotor->EXT_ID.motor_id = hmotor->motor_id;
    // hmotor->EXT_ID.data = FloatToUint(torque,T_MIN,T_MAX,16);
    // hmotor->EXT_ID.res = 0;
    hmotor->motor_can_instance->tx_buff[0]=FloatToUint(MechPosition,P_MIN,P_MAX,16)>>8;
    hmotor->motor_can_instance->tx_buff[1]=FloatToUint(MechPosition,P_MIN,P_MAX,16);
    hmotor->motor_can_instance->tx_buff[2]=FloatToUint(speed,V_MIN,V_MAX,16)>>8;
    hmotor->motor_can_instance->tx_buff[3]=FloatToUint(speed,V_MIN,V_MAX,16);
    hmotor->motor_can_instance->tx_buff[4]=FloatToUint(kp,KP_MIN,KP_MAX,16)>>8;
    hmotor->motor_can_instance->tx_buff[5]=FloatToUint(kp,KP_MIN,KP_MAX,16);
    hmotor->motor_can_instance->tx_buff[6]=FloatToUint(kd,KD_MIN,KD_MAX,16)>>8;
    hmotor->motor_can_instance->tx_buff[7]=FloatToUint(kd,KD_MIN,KD_MAX,16);
    CANTransmit(hmotor->motor_can_instance,1);
    
}

/**
  * @brief          小米电机使能（通信类型 3）
  * @param[in]      hmotor 电机结构体
  * @param[in]      id 电机id
  * @retval         none
  */
 static void MI_motor_Enable(MIMotorInstance* hmotor)
{   
    
    static EXT_ID_T EXT  ;
    EXT.fields.mode=3;
    EXT.fields.data=MI_MASTERID;
    EXT.fields.motor_id=hmotor->motor_can_instance->tx_id;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId= EXT.raw;
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->motor_can_instance->tx_buff[i]=0;
    }
    CANTransmit(hmotor->motor_can_instance,1);
    // if(!CANTransmit(hmotor->motor_can_instance,1)){
    //     while(1){

    //     }
    // }
     
}
/**
  * @brief          电机停止运行帧（通信类型4）
  * @param[in]      hmotor 电机结构体
  * @retval         none
  */
static void MI_motor_Stop(MIMotorInstance* hmotor)

{   
    static EXT_ID_T EXT;
    EXT.fields.mode=4;
    EXT.fields.data=MI_MASTERID;
    EXT.fields.motor_id=hmotor->motor_can_instance->tx_id;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId= EXT.raw;
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->motor_can_instance->tx_buff[i]=0;
    }

    CANTransmit(hmotor->motor_can_instance,1);
}
/**
  * @brief          设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param[in]      hmotor 电机结构体
  * @retval         none
  */
static void MI_motor_SetMechPositionToZero(MIMotorInstance* hmotor)
{
    static EXT_ID_T EXT ;
    EXT.fields.mode=6;
    EXT.fields.data=MI_MASTERID;
    EXT.fields.motor_id=hmotor->motor_can_instance->tx_id;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId= EXT.raw;
    hmotor->motor_can_instance->tx_buff[0]=1;
    for(uint8_t i=1; i<8; i++)
    {
        hmotor->motor_can_instance->tx_buff[i]=0;
    }

    CANTransmit(hmotor->motor_can_instance,1);
     
}

/**
  * @brief          单个参数读取（通信类型17）
  * @param[in]      hmotor 电机结构体
  * @param[in]      index 功能码
  * @retval         none
  */
static void MI_motor_ReadParam(MIMotorInstance* hmotor,uint16_t index)
{
    static EXT_ID_T EXT ;
    EXT.fields.mode=17;
    EXT.fields.data=MI_MASTERID;
    EXT.fields.motor_id=hmotor->motor_can_instance->tx_id;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId= EXT.raw;

    memcpy(&hmotor->motor_can_instance->tx_buff[0],&index,2);
    for(uint8_t i=2; i<8; i++)
    {
        hmotor->motor_can_instance->tx_buff[i]=0;
    }

    CANTransmit(hmotor->motor_can_instance,1);
    // free(EXT);
}

/**
  * @brief          小米电机运行模式切换
  * @param[in]      hmotor 电机结构体
  * @param[in]      run_mode 更改的模式
  * @note           通信类型18 （掉电丢失）
  * @retval         none
  */
static void MI_motor_ModeSwitch(MIMotorInstance* hmotor, uint8_t run_mode,uint16_t index)
{
     
    static EXT_ID_T EXT ;
    EXT.fields.mode=18;
    EXT.fields.data=MI_MASTERID;
    EXT.fields.motor_id=hmotor->motor_can_instance->tx_id;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId=EXT.raw;

    
    for(uint8_t i=2; i<8; i++)
    {
        hmotor->motor_can_instance->tx_buff[i]=0;
    }
    memcpy(&hmotor->motor_can_instance->tx_buff[0],&index,2);
    memcpy(&hmotor->motor_can_instance->tx_buff[4],&run_mode,1);
    CANTransmit(hmotor->motor_can_instance,1);
    // if(!CANTransmit(hmotor->motor_can_instance,1)){
    //     while(1){

    //     }
    // }
}
 void MI_motor_Modeset(MIMotorInstance* hmotor,float param,uint16_t index)
{
    static EXT_ID_T  EXT;
    EXT.fields.mode=18;
    EXT.fields.data=MI_MASTERID;
    EXT.fields.motor_id=hmotor->motor_can_instance->tx_id;
    EXT.fields.res=0;
    hmotor->motor_can_instance->txconf.ExtId= EXT.raw;

    
    for(uint8_t i=2; i<8; i++)
    {
        hmotor->motor_can_instance->tx_buff[i]=0;
    }
    memcpy(&hmotor->motor_can_instance->tx_buff[0],&index,2);
    memcpy(&hmotor->motor_can_instance->tx_buff[4],&param,4);
    CANTransmit(hmotor->motor_can_instance,1);
    // if(!CANTransmit(hmotor->motor_can_instance,1)){
    //     while(1){

    //     }
    // }
}
/*-------------------- 封装的一些控制函数 --------------------*/


/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      torque 目标力矩
  * @retval         none
  */
void  MI_motor_setzero(MIMotorInstance* hmotor)
{   
     MI_motor_SetMechPositionToZero( hmotor);
    //     MI_motor_Control( hmotor, float torque, float MechPosition , float speed , float kp , float kd)
    // }
    // MI_motor_Modeset(hmotor, torque,0x7006);
}
/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      torque 目标力矩
  * @retval         none
  */
void  MI_motor_YunkongControl(MIMotorInstance* hmotor)
{   
    if(hmotor->motor_settings.ifsetflag==0){
    MI_motor_ModeSwitch(hmotor, 0,0x7005);
    MI_motor_Enable ( hmotor);}
    // else{
    //     MI_motor_Control( hmotor, float torque, float MechPosition , float speed , float kp , float kd)
    // }
    // MI_motor_Modeset(hmotor, torque,0x7006);
}
/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      torque 目标力矩
  * @retval         none
  */
void  MI_motor_TorqueControl(MIMotorInstance* hmotor)
{
    MI_motor_ModeSwitch(hmotor, 3,0x7005);
    MI_motor_Enable ( hmotor);
    // MI_motor_Modeset(hmotor, torque,0x7006);
}

/**
  * @brief          小米电机力矩控制模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      torque 目标力矩
  * @retval         none
  */
void  MI_motor_stop(MIMotorInstance* hmotor)
{
    MI_motor_Stop( hmotor);
}

/**
  * @brief          小米电机位置模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      location 控制位置 rad
  * @param[in]      kp 响应速度(到达位置快慢)，一般取1-10
  * @param[in]      kd 电机阻尼，过小会震荡，过大电机会震动明显。一般取0.5左右
  * @retval         none
  */
void  MI_motor_LocationControl(MIMotorInstance* hmotor)
{
   MI_motor_ModeSwitch(hmotor, 1,0x7005);
   osDelay(20);
   MI_motor_Enable ( hmotor);
   osDelay(20);
   MI_motor_Modeset(hmotor, hmotor->motor_controller.speed_PID.MaxOut,0x7017);//最大速度
   osDelay(20);
//    MI_motor_Modeset(hmotor,hmotor->motor_controller.current_PID.MaxOut,0x7018);
//     osDelay(20);
    // MI_motor_Modeset(hmotor, location,0x7016);//速度

}

/**
  * @brief          小米电机速度模式控制指令
  * @param[in]      hmotor 电机结构体
  * @param[in]      speed 控制速度
  * @param[in]      kd 响应速度，一般取0.1-1
  * @retval         none
  */
void  MI_motor_SpeedControl(MIMotorInstance* hmotor)
{
    MI_motor_ModeSwitch(hmotor, 2,0x7005);
   MI_motor_Enable ( hmotor);
   MI_motor_Modeset(hmotor,hmotor->motor_controller.current_PID.MaxOut,0x7018);//最大dianliu
    // MI_motor_Modeset(hmotor,speed,0x700A);//速度
}

// /**
//   * @brief          小米电机电流模式控制指令
//   * @param[in]      hmotor 电机结构体
//   * @param[in]      iq_ref 控制电流
//   * @retval         none
//   */
// void MI_motor_CurrentMode(MI_Motor_s* hmotor, float iq_ref)
// {
//     MI_motor_ModeSwitch(hmotor, CURRENT_MODE);
//     MI_motor_Enable(hmotor);
//     MI_motor_WritePram(hmotor, IQ_REF, RangeRestrict(iq_ref, IQ_REF_MIN, IQ_REF_MAX));
// }