#include "distance.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "stdio.h"
    
#define distance_size 57

// 传感器数据
static Distance_data dt_da[2];      
static uint8_t dt_init_flag = 0; 

static USARTInstance *dt_usart_instance;
static DaemonInstance *dt_daemon_instance;
static uint8_t isProcessing = 0;


 static void data_to_distance(const uint8_t*dt_data)
 {      

    
    // const char *frame_header1 = "1A"; // 假设帧头是 "HEADER"
    // const char *frame_header2 = "2A";
    // const char *frame_header3 = "3A";
    // char header_buffer[3]; // 存储帧头的缓冲区
    // strncpy(header_buffer, (const char *)dt_data, 2); // 复制前5个字符到缓冲区
    // header_buffer[3] = '\0'; // 确保字符串正确结束

    // // 检查帧头是否正确
         
    //   if (strcmp(header_buffer, frame_header1) == 0) {
    //     // 跳过帧头部分，解析后面的数据
    //     const char *data_start = (const char *)dt_data + 2;
    //     dt_da[TEMP].type = 1;
        
    //     // 使用缓冲区来存储数据
    //     char buffer[48]; // 每个数据块的最大长度
    //     strncpy(buffer,  data_start, sizeof(buffer) - 1);
    //     buffer[sizeof(buffer) - 1] = '\0'; // 确保字符串以 null 结尾
    
        if(dt_data[0]==49&&dt_data[1]==65&&dt_data[8]==83){//第一种通讯方式给参考值
            dt_da->type=1;
            static int ID,reverse,ref;
            sscanf(dt_data, "1A%2d%4dS\n",
                &ID,&ref); // 将数据转换为整型
            if(ID==11){
              
                  dt_da->type1.ref1=ref;
            }
            if(ID==12){
                
                dt_da->type1.ref2=ref;
            }
            if(ID==13){
                
                dt_da->type1.ref3=ref;
            }
            if(ID==21){
              
                dt_da->type1.ref4=ref;
            }
            if(ID==22){
              
                dt_da->type1.ref5=ref;
            }
            if(ID==23){
            
                dt_da->type1.ref6=ref;
            }
            if(ID==31){
              
                dt_da->type1.ref7=ref;
            }
            if(ID==32){
           
                dt_da->type1.ref8=ref;
            }
            if(ID==33){
               
                dt_da->type1.ref9=ref;
            }
            if(ID==41){
             
                dt_da->type1.ref10=ref;
            }
            if(ID==42){
             
                dt_da->type1.ref11=ref;
            }
            if(ID==43){
                
                dt_da->type1.ref12=ref;
            }
            memcpy(&dt_da[LAST], &dt_da[TEMP], sizeof(Distance_data));
        }else
        if(dt_data[0]==50&&dt_data[1]==65&&dt_data[8]==83){
            dt_da->type=2;
             sscanf(dt_data, "%2A2d%2d%2dS\n",
               &dt_da[TEMP].type2.id1, &dt_da[TEMP].type2.id2, &dt_da[TEMP].type2.id3); // 将数据转换为整型

        memcpy(&dt_da[LAST], &dt_da[TEMP], sizeof(Distance_data));
        }
    // } else if (strcmp(header_buffer, frame_header2) == 0) {
         
    //     dt_da[TEMP].type = 2;
    //     const char *data_start = (const char *)dt_data + 2;

    //     // 使用缓冲区来存储数据
    //     char buffer[6]; // 每个数据块的最大长度
    //     strncpy(buffer, data_start, sizeof(buffer) );
    //     buffer[sizeof(buffer) ] = '\0'; // 确保字符串以 null 结尾

    //     sscanf(buffer, "%2d%2d%2d\n",
    //            &dt_da[TEMP].type2.id1, &dt_da[TEMP].type2.id2, &dt_da[TEMP].type2.id3); // 将数据转换为整型

    //     memcpy(&dt_da[LAST], &dt_da[TEMP], sizeof(Distance_data));
         
    // } else if (strcmp(header_buffer, frame_header3) == 0) {
         
    //     dt_da[TEMP].type = 3;
    //     const char *data_start = (const char *)dt_data + 2;

    //     // 使用缓冲区来存储数据
    //     char buffer[36]; // 每个数据块的最大长度
    //     strncpy(buffer, data_start, sizeof(buffer) - 1);
    //     buffer[sizeof(buffer) - 1] = '\0'; // 确保字符串以 null 结尾

    //     sscanf(buffer, "%2d%1d\n",
    //            &dt_da[TEMP].type3.id, &dt_da[TEMP].type3.mode
    //            ); // 将数据转换为整型

    //     memcpy(&dt_da[LAST], &dt_da[TEMP], sizeof(Distance_data));
        
    // }
 }

 static void dtRxCallback()
{   
    DaemonReload(dt_daemon_instance);  
// && dt_usart_instance->recv_buff[56]== "S" && dt_usart_instance->recv_buff[55]== "E"&&dt_usart_instance->recv_buff[54]== "R"  
//    if((dt_usart_instance->recv_buff[1]==65)&&(dt_usart_instance->recv_buff[53]==65)) {
    // HAL_NVIC_DisableIRQ(USART1_IRQn);
           

    data_to_distance(dt_usart_instance->recv_buff);
    // HAL_NVIC_EnableIRQ(USART1_IRQn);  

 
}

 static void dtLostCallback(void *id)
{
    memset(dt_da, 0, sizeof(dt_da)); // 清空遥控器数据
    USARTServiceInit(dt_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}


 Distance_data *dtInit(UART_HandleTypeDef *dt_usart_handle){
    USART_Init_Config_s conf;
    conf.module_callback = dtRxCallback;
    conf.usart_handle = dt_usart_handle;
    conf.recv_buff_size = 9 ;
    dt_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 100, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = dtLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    dt_daemon_instance = DaemonRegister(&daemon_conf);

    dt_init_flag = 1;
    return dt_da;
 }
 