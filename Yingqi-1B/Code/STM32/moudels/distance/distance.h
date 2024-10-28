/**
 * @file distance.h
 * @author sive
 * @brief  测距传感器定义头文件
 * @version beta
 * @date 2024-4-11
 *
 */
 
#include <stdint.h>
#include "main.h"
#include "usart.h"

#define LAST 1
#define TEMP 0


typedef struct  
{
    /* data */
    int reverse1;
    int ref1;
    int reverse2;
    int ref2;
    int reverse3;
    int ref3;
    int reverse4;
    int ref4;
    int reverse5;
    int ref5;
    int reverse6;
    int ref6;
    int reverse7;
    int ref7;
    int reverse8;
    int ref8;
    int reverse9;
    int ref9;
    int reverse10;
    int ref10;
    int reverse11;
    int ref11;
    int reverse12;
    int ref12;

}typedata1;
typedef struct  
{
    /* data */
    int id1;
    int id2;
    int id3;
}typedata2;
typedef struct  
{
    /* data */
    int id;
    int mode;
    int data1;
    int data2;
    int data3;
    int data4;
}typedata3;

typedef struct 
{
     int type;
     typedata1 type1;
     typedata2 type2;
     typedata3 type3;

    /* data */
} Distance_data ;

 Distance_data *dtInit(UART_HandleTypeDef *dt_usart_handle);

  