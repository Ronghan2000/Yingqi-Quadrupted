/*
   -- SUSDOG-S3 --

   This source code of graphical user interface
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/
     - for ANDROID 4.13.13 or later version;
     - for iOS 1.10.3 or later version;

   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "SUSDOG"
#define REMOTEXY_WIFI_PASSWORD "ARES4EVER"
#define REMOTEXY_SERVER_PORT 6377


#include <RemoteXY.h>

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 212 bytes
{ 255, 9, 0, 0, 0, 205, 0, 17, 0, 0, 0, 31, 2, 106, 200, 200, 126, 1, 1, 13,
  0, 3, 35, 62, 26, 70, 8, 33, 49, 13, 132, 163, 26, 10, 33, 91, 38, 38, 106, 99,
  15, 15, 48, 178, 31, 31, 65, 0, 178, 77, 0, 10, 37, 135, 38, 38, 106, 78, 15, 15,
  48, 38, 31, 31, 79, 78, 0, 52, 79, 70, 70, 0, 1, 45, 73, 38, 38, 116, 57, 17,
  17, 0, 64, 31, 74, 117, 109, 112, 33, 0, 4, 22, 86, 6, 116, 21, 52, 11, 73, 32,
  120, 26, 4, 5, 86, 6, 116, 7, 52, 11, 73, 0, 163, 26, 4, 6, 89, 6, 116, 35,
  52, 11, 73, 0, 178, 26, 4, 7, 92, 6, 116, 50, 52, 11, 73, 0, 220, 26, 3, 29,
  3, 26, 70, 159, 23, 13, 93, 8, 2, 26, 129, 42, 40, 46, 19, 129, 23, 28, 12, 17,
  70, 114, 111, 110, 116, 0, 129, 54, 40, 45, 19, 132, 48, 21, 12, 17, 76, 101, 102, 116,
  0, 129, 60, 79, 34, 19, 132, 69, 27, 12, 17, 66, 97, 99, 107, 0, 129, 61, 83, 34,
  19, 130, 92, 28, 12, 17, 82, 105, 103, 104, 116, 0
};

// this structure defines all the variables and events of your control interface
struct {

  // input variables
  uint8_t select_01; // from 0 to 4
  uint8_t joystick; // =1 if state is ON, else =0
  uint8_t power; // =1 if state is ON, else =0
  uint8_t jump; // =1 if button pressed, else =0
  int8_t t; // from -100 to 100
  int8_t w; // from 0 to 100
  int8_t jh; // from 0 to 100
  int8_t jx; // from 0 to 100
  uint8_t select_02; // from 0 to 8

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

int executed2 = 0;

void intToThreeDigitString(int num, char *str)
{
  snprintf(str, 4, "%03d", num);
}
int agi = 0;
int dhi = 0;
int dwi = 0;
int ti = 0;
int wi = 0;
int jti = 0;
int jxi = 0;
int statei = 0;
char agc[4];
char dhc[4];
char dwc[4];
char tc[4];
char wc[4];
char jtc[4];
char jxc[4];
char statec[4];

//void autoControlCommand()
//{
//  // while (1)
//  // {
//  intToThreeDigitString(statei, statec);
//  intToThreeDigitString((RemoteXY.joyx + 100) / 2, agc);
//  intToThreeDigitString((RemoteXY.joyy + 100) / 2, dhc);
//  //    intToThreeDigitString(dwi, dwc);
//  //    intToThreeDigitString(ti, tc);
//  //    intToThreeDigitString(wi, wc);
//  intToThreeDigitString(RemoteXY.jh / 2, jtc);
//  intToThreeDigitString(RemoteXY.jx / 2, jxc);
//
//  Serial.println("A" + String(statec) + String(agc) + String(dhc) + String(jtc) + String(jxc) + "S");
//  //  Serial1.println("A" + String(statec) + String(agc) + String(dhc) + String(jtc) + String(jxc) + "S");
//
//  // }
//}

void setup()
{
  pinMode(00, OUTPUT);
  pinMode(01, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(06, OUTPUT);
  pinMode(13, OUTPUT);
  RemoteXY_Init();
  RemoteXY_delay(50);

  Serial.begin(115200);
  //  Serial1.begin(115200);

  // TODO you setup code
}

void loop()
{
  RemoteXY_Handler();
  digitalWrite(13, RemoteXY.connect_flag);
//  digitalWrite(12, !RemoteXY.jump);

  if (RemoteXY.select_01 == 0)
  {
    digitalWrite(00, HIGH);
    digitalWrite(01, HIGH);
    digitalWrite(06, HIGH);
    //    Serial.println("1");
  }
  else if (RemoteXY.select_01 == 1)
  {
    executed2 = 1;
    digitalWrite(00, LOW);
    digitalWrite(01, HIGH);
    digitalWrite(06, HIGH);
    //    Serial.println("2");
    //    RemoteXY_delay(5);
    // digitalWrite(00, HIGH);
    // digitalWrite(01, HIGH);
    // digitalWrite(06, HIGH);
    statei = 1;
  }
  else if (RemoteXY.select_01 == 2)
  {
    digitalWrite(00, HIGH);
    digitalWrite(01, LOW);
    digitalWrite(06, HIGH);
    //    Serial.println("3");
    statei = 2;
  }
  else if (RemoteXY.select_01 == 3)
  {
    digitalWrite(00, HIGH);
    digitalWrite(01, HIGH);
    digitalWrite(06, LOW);
    //    Serial.println("4");
    statei = 3; 
  }
  else if (RemoteXY.select_01 == 0)
  {
    digitalWrite(00, HIGH);
    digitalWrite(01, HIGH);
    digitalWrite(06, HIGH);
    //    Serial.println("0");
    statei = 0;
  }
  //  agi = RemoteXY.joyx/2;
  //  dhi = RemoteXY.joyy/2;
  //  dwi = RemoteXY.dw/2;
  //  ti = RemoteXY.t/2;
  //  wi = RemoteXY.w/2;
  //  jti = RemoteXY.jh/2;
  //  jxi = RemoteXY.jx/2;
  //  if (！Serial.available()) {
  //    Serial.begin(115200);
  //  }

  //  autoControlCommand();
  Serial.println(RemoteXY.select_02);



  RemoteXY_delay(5);

  // do not call delay(), use instead RemoteXY_delay()
}