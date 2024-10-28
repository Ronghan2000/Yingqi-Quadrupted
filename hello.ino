#include <Arduino.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#include <TeensyThreads.h>
#include <Bounce2.h>

// 贝内特机构正逆运动学部分

#define PI 3.1415926535897931

#define l1 0.060
#define l2 0.130
#define l3 0.130
#define l4 0.000

#define UART1_BaudRate 1000000
#define UART2_BaudRate 1000000
#define UART3_BaudRate 115200

constexpr double M_SQRT1_3 = 1 / sqrt(3);
constexpr float DEG2RAD = PI / 180;
constexpr float RAD2DEG = 180 / PI;
constexpr double af = PI / 4;
constexpr double bt = 3 * PI / 4;

const double saf = sin(af);
const double caf = cos(af);
const double sbt = sin(bt);
const double cbt = cos(bt);

// static

int ErrorFlag = 0;
static volatile int hasStandup = 0;
static volatile int isUp = 0;
static volatile int state = 0; // 0=default,1=stand & sit,2=trot,3=jump
static volatile int Direction = 0;

static volatile float faai = 0.4;    // 占空比
static volatile float tsf = 0.4;     // 单步时间
static volatile float h = 0.05;      // 抬腿高度
static volatile float veloAngle = 0; // 速度方向
static volatile float dh = 0.01;      // 走路时的躯体高度变化，基于-0.15
static volatile float dw = 0.0;      // 走路时的左右两足间距变化，基于0.10
static volatile float w = 0.04;      // 单步步长的一半，默认0.08
static volatile float jx = 0.08;
static volatile float lf = 0.00;
static volatile float rf = 0.00;

static BLA::Matrix<3, 1> sitdownPos = {0.016, 0.10, -0.02};
static BLA::Matrix<3, 1> standupPos = {0.03, 0.12, -0.12};
static BLA::Matrix<3, 1> lowPos = {0.03, 0.11, -0.06};
static BLA::Matrix<3, 1> jumpPosf = {0.03 - jx, 0.11, -0.18};
static BLA::Matrix<3, 1> jumpPosr = {0.03 + jx, 0.11, -0.18};

// 以下是为了提速预留的变量
float sigma = 0.0;
float zep = 0.0;
float xep_b = 0.0;
float xep_z = 0.0;
float d = 0.0;//转向角度
unsigned long trotStart = 0;

using namespace BLA;

BLA::Matrix<6, 1> S1 = {1, 0, 0, 0, 0, 0};
BLA::Matrix<6, 1> S2 = {0, 1, 0, 0, 0, 0};
BLA::Matrix<6, 1> S3 = {sbt, cbt, 0, l3 * cbt, -l3 * cbt, -l1 * sbt};

BLA::Matrix<3, 1> p;
BLA::Matrix<3, 1> p2;
BLA::Matrix<3, 1> fl;
BLA::Matrix<3, 1> fr;
BLA::Matrix<3, 1> rl;
BLA::Matrix<3, 1> rr;
BLA::Matrix<3, 1> FL;
BLA::Matrix<3, 1> FR;
BLA::Matrix<3, 1> RL;
BLA::Matrix<3, 1> RR;

BLA::Matrix<3, 3> times3(double t)
{
  BLA::Matrix<3, 3> I;
  I = {t, 0, 0,
       0, t, 0,
       0, 0, t
      };
  return I;
}

// the function below is used to generate the 4x4 identity matrix multiplied by a scalar
BLA::Matrix<4, 4> times4(double t)
{
  BLA::Matrix<4, 4> I;
  I = {t, 0, 0, 0,
       0, t, 0, 0,
       0, 0, t, 0,
       0, 0, 0, t
      };
  return I;
}

// the function below is used to generate the 3x3 skew symmetric matrix of a 3x1 vector
BLA::Matrix<3, 3> SkewSemmetric(BLA::Matrix<3, 1> v)
{
  BLA::Matrix<3, 3> S;
  S(0, 0) = 0;
  S(0, 1) = -v(2);
  S(0, 2) = v(1);
  S(1, 0) = v(2);
  S(1, 1) = 0;
  S(1, 2) = -v(0);
  S(2, 0) = -v(1);
  S(2, 1) = v(0);
  S(2, 2) = 0;
  return S;
}

// the function below is used to generate the 3x3 exponential matrix of a 3x1 vector and an angle scalar
BLA::Matrix<3, 3> expm3(BLA::Matrix<3, 1> w, double t)
{
  BLA::Matrix<3, 3> S = SkewSemmetric(w);
  BLA::Matrix<3, 3> E;
  E = times3(1.0) + times3(sin(t)) * S + times3(1.0 - cos(t)) * S * S;
  return E;
}

// the function below is used to generate the 4x4 screw matrix of a 6x1 screw vector
BLA::Matrix<4, 4> Screw44(BLA::Matrix<6, 1> q)
{
  BLA::Matrix<3, 3> w;
  BLA::Matrix<3, 1> v;
  BLA::Matrix<1, 4> Z;
  w = SkewSemmetric(q.Submatrix<3, 1>(0, 0));
  v = q.Submatrix<3, 1>(3, 0);
  Z.Fill(0);
  return (w || v) && Z;
}

// the function below is used to generate the 4x4 exponential matrix of a 6x1 screw vector and an angle scalar
BLA::Matrix<4, 4> expm4(BLA::Matrix<6, 1> S, double t)
{
  BLA::Matrix<3, 1> w = S.Submatrix<3, 1>(0, 0);
  BLA::Matrix<3, 1> v = S.Submatrix<3, 1>(3, 0);
  BLA::Matrix<3, 3> W = expm3(w, t);
  BLA::Matrix<3, 3> SkW = SkewSemmetric(w);
  BLA::Matrix<3, 1> p = (times3(t) + times3(1 - cos(t)) * SkW + times3(t - sin(t)) * SkW * SkW) * v;
  BLA::Matrix<1, 4> Z = {0, 0, 0, 1};
  BLA::Matrix<4, 4> E = (W || p) && Z;
  return E;
}

// the function below is used to generate
BLA::Matrix<6, 6> Adjoint(BLA::Matrix<4, 4> T)
{
  BLA::Matrix<3, 3> R = T.Submatrix<3, 3>(0, 0);
  BLA::Matrix<3, 1> p = T.Submatrix<3, 1>(0, 3);
  BLA::Matrix<3, 3> p_hat = SkewSemmetric(p);
  BLA::Matrix<6, 6> Ad;
  Ad = (R || times3(0.0)) && (p_hat * R || R);
  return Ad;
}

// Inverse Kinematics
BLA::Matrix<3, 1> IK(BLA::Matrix<3, 1> p)
{
  BLA::Matrix<3, 1> q;
  double a5 = 2 * l1 * (l2 + l4) * sin(bt);
  double b5 = 2 * l3 * (l2 + l4);
  double c5 = l1 * l1 + l3 * l3 + (l2 + l4) * (l2 + l4) - p(0) * p(0) - p(1) * p(1) - p(2) * p(2);

  double q51 = atan2(-c5, sqrt(a5 * a5 + b5 * b5 - c5 * c5)) - atan2(b5, a5);
  double q52 = atan2(-c5, -sqrt(a5 * a5 + b5 * b5 - c5 * c5)) - atan2(b5, a5);
  if (fabs(q51) < 2.3)
  {
    q(2) = q51;
  }
  // else if (fabs(q51) < 2.3)
  // {
  //   q(2) = q51;
  // }
  else
  {
    Serial.println("P3 No solution");
    ErrorFlag = 1;
  }

  double a4 = l3 + (l2 + l4) * cos(q(2));
  double b4 = (l2 + l4) * cos(bt) * sin(q(2));
  double c4 = p(0);

  double q41 = atan2(-c4, sqrt(a4 * a4 + b4 * b4 - c4 * c4)) - atan2(b4, a4);
  double q42 = atan2(-c4, -sqrt(a4 * a4 + b4 * b4 - c4 * c4)) - atan2(b4, a4);

  if (2 * fabs(q41) < PI)
  {
    q(1) = q41;
  }
  // else if (2 * fabs(q42) < PI)
  // {
  //   q(1) = q42;
  // }
  else
  {
    Serial.println("P2 No solution");
    ErrorFlag = 1;
  }

  double a1 = -(l2 + l4) * cos(bt) * sin(q(1)) * sin(q(2)) + (l2 + l4) * cos(q(1)) * cos(q(2)) + l3 * cos(q(1));
  double b1 = l1 + (l2 + l4) * sin(bt) * sin(q(2));

  q(0) = asin((p(1) * a1 + p(2) * b1) / (p(1) * p(1) + p(2) * p(2)));

  return q;
}

void blinkthread()
{
  while (1)
  {
    digitalWrite(13, HIGH);
    threads.delay(150);
    digitalWrite(13, LOW);
    threads.delay(150);
  }
}

BLA::Matrix<3, 1> Correction4Robot(BLA::Matrix<3, 1> p0)
{
  BLA::Matrix<3, 1> p1 = IK(p0) * RAD2DEG;
  // Serial << "FC: " << p1 << '\n';

  BLA::Matrix<3, 1> p2;
  p2(0) = p1(0) + 6;
  if (p1(1) >= -67)
  {
    p2(1) = p1(1) + 67;
  }
  else
  {
    ErrorFlag = 1;
    Serial.println("P2 Out of range");
    threads.addThread(blinkthread);
  }
  p2(1) = p1(1) + 67;

  if (p1(2) >= -133 && p1(2) <= -30)
  {
    p2(2) = p1(2) + 133;
  }
  else
  {
    ErrorFlag = 1;
    Serial.println("P3 Out of range");
    threads.addThread(blinkthread);
  }
  // Serial << "FD: " << p2 << '\n';
  return p2;
}

BLA::Matrix<6, 3> GeometricalJacobian(double q1, double q4)
{
  BLA::Matrix<6, 1> Jg2 = Adjoint(expm4(S1, q1)) * S2;
  BLA::Matrix<6, 1> Jg3 = Adjoint(expm4(S1, q1) * expm4(S2, q4)) * S3;
  BLA::Matrix<6, 3> Jg = (S1 || Jg2 || Jg3);
  return Jg;
}

BLA::Matrix<3, 3> AnalyticalJacobian(double q1, double q4, BLA::Matrix<3, 1> p)
{
  BLA::Matrix<3, 6> J = (times3(-1.0) * SkewSemmetric(p)) || (times3(1.0));
  BLA::Matrix<3, 3> Ja = J * GeometricalJacobian(q1, q4);
  return Ja;
}

// 通信部分
// 我知道用二维数组然后for循环很优雅，但这样摊平了更快，快多了

static volatile int pos11 = 0;
static volatile int pos12 = 0;
static volatile int pos13 = 0;
static volatile int pos21 = 0;
static volatile int pos22 = 0;
static volatile int pos23 = 0;
static volatile int pos31 = 0;
static volatile int pos32 = 0;
static volatile int pos33 = 0;
static volatile int pos41 = 0;
static volatile int pos42 = 0;
static volatile int pos43 = 0;

BLA::Matrix<3, 1> cartisianPos = {0, 0, 0};

// 定义电机结构体
struct MotorData
{
  uint8_t motor_id;
  float angle_single_round;
  float speed_aps;
  float real_current;
};

unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
const long interval1 = 10; // 任务1的时间间隔
const long interval2 = 2;  // 任务2的时间间隔

// 定义电机实例
MotorData motors[12];

const int bufferSize = 15;
uint8_t receiveBuffer[bufferSize];
int bufferIndex = 0;
const int ringBufferSize = 64;
uint8_t ringBuffer[ringBufferSize];
int ringBufferStart = 0;
int ringBufferEnd = 0;

void storeMotorData(uint8_t motor_id, float angle, float speed, float current)
{ // 接受处理
  int index = motor_id - 1; // 根据 motor_id 计算数组索引，motor_id 从1开始
  if (index >= 0 && index < 12)
  {
    motors[index].motor_id = motor_id;
    motors[index].angle_single_round = angle;
    motors[index].speed_aps = speed;
    motors[index].real_current = current;
  }
  else
  {
    // Serial.println("Invalid motor ID");
  }
}
void getdata()
{
  while (Serial1.available())
  {
    uint8_t incomingByte = Serial1.read();

    // 将数据写入环形缓冲区
    ringBuffer[ringBufferEnd] = incomingByte;
    ringBufferEnd = (ringBufferEnd + 1) % ringBufferSize;

    // 检查缓冲区是否已满
    if (ringBufferEnd == ringBufferStart)
    {
      // 缓冲区溢出，处理溢出情况（例如，丢弃最旧的数据）
      ringBufferStart = (ringBufferStart + 1) % ringBufferSize;
    }
  }

  // 检查是否可以读取一个完整的数据包
  while ((ringBufferEnd + ringBufferSize - ringBufferStart) % ringBufferSize >= bufferSize)
  {
    // 从环形缓冲区读取数据包
    for (int i = 0; i < bufferSize; i++)
    {
      receiveBuffer[i] = ringBuffer[(ringBufferStart + i) % ringBufferSize];
    }
    ringBufferStart = (ringBufferStart + bufferSize) % ringBufferSize;

    // 解析数据包
    Serial.print("1 ");
    if (receiveBuffer[0] == 'A' && receiveBuffer[14] == 'S')
    {
      uint8_t motor_id = receiveBuffer[1];
      float angle_single_round;
      float speed_aps;
      float real_current;

      memcpy(&angle_single_round, &receiveBuffer[2], 4);
      memcpy(&speed_aps, &receiveBuffer[6], 4);
      memcpy(&real_current, &receiveBuffer[10], 4);

      // 根据 motor_id 将数据存储到对应的电机实例中
      storeMotorData(motor_id, angle_single_round, speed_aps, real_current);

      // 打印解析后的数据
      Serial.print("Motor ID: ");
      Serial.println(motor_id);
      Serial.print("Angle (Single Round): ");
      Serial.println(angle_single_round);
      Serial.print("Speed (APS): ");
      Serial.println(speed_aps);
      Serial.print("Real Current: ");
      Serial.println(real_current);
    }
    else
    {
      // 如果数据包无效，可以打印错误信息或者采取其他措施
      Serial.println("Invalid data packet received");
    }
  }
}

void intToThreeDigitString(int num, char *str)
{
  snprintf(str, 4, "%03d", num);
}

void autoControlCommand()
{
  while (1)
  {
    char pos11s[4];
    char pos12s[4];
    char pos13s[4];
    char pos21s[4];
    char pos22s[4];
    char pos23s[4];
    char pos31s[4];
    char pos32s[4];
    char pos33s[4];
    char pos41s[4];
    char pos42s[4];
    char pos43s[4];

    intToThreeDigitString(pos11, pos11s);
    intToThreeDigitString(pos12, pos12s);
    intToThreeDigitString(pos13, pos13s);
    intToThreeDigitString(pos21, pos21s);
    intToThreeDigitString(pos22, pos22s);
    intToThreeDigitString(pos23, pos23s);
    intToThreeDigitString(pos31, pos31s);
    intToThreeDigitString(pos32, pos32s);
    intToThreeDigitString(pos33, pos33s);
    intToThreeDigitString(pos41, pos41s);
    intToThreeDigitString(pos42, pos42s);
    intToThreeDigitString(pos43, pos43s);

    // Serial.println("1A11+" + String(pos11s) + "S");
    // Serial.println("1A12+" + String(pos12s) + "S");
    // Serial.println("1A13+" + String(pos13s) + "S");
    // Serial.println("1A21+" + String(pos21s) + "S");
    // Serial.println("1A22+" + String(pos22s) + "S");
    // Serial.println("1A23+" + String(pos23s) + "S");
    // Serial.println("1A31+" + String(pos31s) + "S");
    // Serial.println("1A32+" + String(pos32s) + "S");
    // Serial.println("1A33+" + String(pos33s) + "S");
    // Serial.println("1A41+" + String(pos41s) + "S");
    // Serial.println("1A42+" + String(pos42s) + "S");
    // Serial.println("1A43+" + String(pos43s) + "S");

    Serial2.println("1A11+" + String(pos11s) + "S");
    Serial2.println("1A12+" + String(pos12s) + "S");
    Serial2.println("1A13+" + String(pos13s) + "S");
    Serial2.println("1A21+" + String(pos21s) + "S");
    Serial2.println("1A22+" + String(pos22s) + "S");
    Serial2.println("1A23+" + String(pos23s) + "S");
    Serial2.println("1A31+" + String(pos31s) + "S");
    Serial2.println("1A32+" + String(pos32s) + "S");
    Serial2.println("1A33+" + String(pos33s) + "S");
    Serial2.println("1A41+" + String(pos41s) + "S");
    Serial2.println("1A42+" + String(pos42s) + "S");
    Serial2.println("1A43+" + String(pos43s) + "S");
    // threads.delay(1);
  }
}

void sendZeroPointCommand(uint8_t motor_id1, uint8_t motor_id2, uint8_t motor_id3)
{
  char command[12];
  snprintf(command, sizeof(command), "2A%02d%02d%02dS", motor_id1, motor_id2, motor_id3);

  // 确保指令长度正确
  if (strlen(command) == 9)
  {
    // Serial.print("Sending zero point command: ");
    Serial.println(command);
    // Serial.write(command, strlen(command));
  }
  else
  {
    // Serial.println("Invalid zero point command length");
  }
}
void sendMovementModeCommand(uint8_t motor_id, uint8_t mode)
{
  char command[9]; // 3ARES + 两位ID + 一位数模式，共8个字符，外加一个终止符
  snprintf(command, sizeof(command), "3ARES%02d%d", motor_id, mode);

  // 确保指令长度正确
  if (strlen(command) == 8)
  {
    // Serial.print("Sending movement mode command: ");
    Serial.println(command);
    // Serial.write(command, strlen(command));
  }
  else
  {
    // Serial.println("Invalid movement mode command length");
  }
}

static void transitionPos(unsigned long transitionTime, int tp11 = 0, int tp12 = 0, int tp13 = 0, int tp21 = 0, int tp22 = 0, int tp23 = 0, int tp31 = 0, int tp32 = 0, int tp33 = 0, int tp41 = 0, int tp42 = 0, int tp43 = 0)
{
  int cv11 = pos11;
  int cv12 = pos12;
  int cv13 = pos13;
  int cv21 = pos21;
  int cv22 = pos22;
  int cv23 = pos23;
  int cv31 = pos31;
  int cv32 = pos32;
  int cv33 = pos33;
  int cv41 = pos41;
  int cv42 = pos42;
  int cv43 = pos43;
  unsigned long startTime = millis();
  float percent = 0.0;
  while (percent < 1.0)
  {
    percent = float(millis() - startTime) / transitionTime;
    pos11 = cv11 + (tp11 - cv11) * percent;
    pos12 = cv12 + (tp12 - cv12) * percent;
    pos13 = cv13 + (tp13 - cv13) * percent;
    pos21 = cv21 + (tp21 - cv21) * percent;
    pos22 = cv22 + (tp22 - cv22) * percent;
    pos23 = cv23 + (tp23 - cv23) * percent;
    pos31 = cv31 + (tp31 - cv31) * percent;
    pos32 = cv32 + (tp32 - cv32) * percent;
    pos33 = cv33 + (tp33 - cv33) * percent;
    pos41 = cv41 + (tp41 - cv41) * percent;
    pos42 = cv42 + (tp42 - cv42) * percent;
    pos43 = cv43 + (tp43 - cv43) * percent;
    threads.delay(1);
  }
  pos11 = tp11;
  pos12 = tp12;
  pos13 = tp13;
  pos21 = tp21;
  pos22 = tp22;
  pos23 = tp23;
  pos31 = tp31;
  pos32 = tp32;
  pos33 = tp33;
  pos41 = tp41;
  pos42 = tp42;
  pos43 = tp43;
}

static void changeState(int i = 1)
{
  state = i;
}

static void standup(int a1 = 90)
{
  if (!hasStandup)
  {
    transitionPos(1000, a1, 0, 0, a1, 0, 0, a1, 0, 0, a1, 0, 0);
    isUp = 0;
  }
  if (isUp == 0)
  {
    // cartisianPos = sitdownPos;
    p = Correction4Robot(sitdownPos);
    transitionPos(300, p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2));
    threads.delay(500);
  }
  cartisianPos = standupPos;
  p = Correction4Robot(standupPos);
  transitionPos(1000, p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2));
  hasStandup = 1;
  isUp = 1;
  Serial.println("Stand UP!");
  changeState(1);
}
static void sitdown(int a1 = 90)
{
  if (isUp != 0)
  {
    // cartisianPos = standupPos;
    p = Correction4Robot(standupPos);
    transitionPos(2000, p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2));
    threads.delay(200);
  }
  // cartisianPos = sitdownPos;
  p = Correction4Robot(sitdownPos);
  transitionPos(1000, p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2));
  transitionPos(500, a1, 0, 0, a1, 0, 0, a1, 0, 0, a1, 0, 0);
  isUp = 0;
  Serial.println("Sit down!");
}

static void jump(int t)
{
  p = Correction4Robot(lowPos);
  Serial.println("Warning!");
  transitionPos(500, p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2));
  threads.delay(200);
  p = Correction4Robot(jumpPosf);
  p2 = Correction4Robot(jumpPosr);
  transitionPos(t, p(0), p(1), p(2), p(0), p(1), p(2), p2(0), p2(1), p2(2), p2(0), p2(1), p2(2));
  threads.delay(10);
  p = Correction4Robot(standupPos);
  transitionPos(t / 2, p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2), p(0), p(1), p(2));
  threads.delay(50);
  Serial.println("done!");
}

// 步态部分

BLA::Matrix<4, 2> Trot(float t, float xs, float xf, float h, int r1, int r4, int r2, int r3)
{
  BLA::Matrix<4, 2> Tr = {0, 0, 0, 0, 0, 0, 0, 0};
  if (t <= tsf * faai)
  {
    sigma = 2 * PI * t / (tsf * faai);
    zep = h / 2 * (1 - cos(sigma));
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;
    Tr = {xep_z * r1, zep,
          xep_b * r2, 0,
          xep_z * r3, zep,
          xep_b * r4, 0
         };
  }
  else if (t > tsf * faai && t < tsf)
  {
    sigma = 2 * PI * (t - tsf * faai) / (tsf * (1 - faai));
    zep = h / 2 * (1 - cos(sigma));
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;
    Tr = {xep_b * r1, 0,
          xep_z * r2, zep,
          xep_b * r3, 0,
          xep_z * r4, zep
         };
  }
  //  Serial << "Tr: " << Tr << '\n';
  return Tr;
}

static void trotPlanner()
{
  trotStart = millis();
  if (state == 2)
  {
    Serial.println("Trot Start!");
  }

  while (state == 2)
  {
    float t = fmod(((millis() - trotStart) / 1000.0), tsf);
    // Serial.println(t);

    BLA::Matrix<4, 2> Trlf = Trot(t, -w - lf, w + lf, h, 1, 1, 1, 1);
    BLA::Matrix<4, 2> Trrf = Trot(t, -w - rf, w + rf, h, 1, 1, 1, 1);
    BLA::Matrix<4, 2> Trlr = Trot(t, -w - lf, w + lf, h+dh, 1, 1, 1, 1);
    BLA::Matrix<4, 2> Trrr = Trot(t, -w - rf, w + rf, h+dh, 1, 1, 1, 1);
    FL = {(standupPos(0) + Trlf(0, 0) * cos(veloAngle)),
          (standupPos(1) + dw + Trlf(0, 0) * sin(veloAngle)),
          (standupPos(2) + dh + Trlf(0, 1))
         };
    FR = {(standupPos(0) + Trrf(1, 0) * cos(veloAngle)),
          (standupPos(1) + dw - Trrf(1, 0) * sin(veloAngle)),
          (standupPos(2) + dh + Trrf(1, 1))
         };
    RL = {(standupPos(0) - Trlr(3, 0) * cos(veloAngle)),
          (standupPos(1) + dw + Trlr(3, 0) * sin(veloAngle)),
          (standupPos(2) + dh + Trlr(3, 1))
         };
    RR = {(standupPos(0) - Trrr(2, 0) * cos(veloAngle)),
          (standupPos(1) + dw - Trrr(2, 0) * sin(veloAngle)),
          (standupPos(2) + dh + Trrr(2, 1))
         };
    //    Serial << "FL: " << FL << '\n';
    fl = Correction4Robot(FL);
    fr = Correction4Robot(FR);
    rl = Correction4Robot(RL);
    rr = Correction4Robot(RR);
    transitionPos(20, fl(0), fl(1), fl(2), fr(0), fr(1), fr(2), rl(0), rl(1), rl(2), rr(0), rr(1), rr(2));
  }
  if (state != 2)
  {
    Serial.println("Trot End!");
    threads.yield();
  }
}

static void TurnPlanner()
{
  trotStart = millis();
  if (state == 5)
  {
    Serial.println("Turn Start!");
  }

  while (state == 5)
  {
    float t = fmod(((millis() - trotStart) / 1000.0), tsf);
    // Serial.println(t);

    BLA::Matrix<4, 2> Trl = Trot(t, -w*0.7 , w*0.7 , h, 1, 1, 1, 1);
    BLA::Matrix<4, 2> Trr = Trot(t, -w*0.7 , w*0.7 , h, 1, 1, 1, 1);
    FL = {(standupPos(0) + Trl(0, 0) * cos(d)),
          (standupPos(1) + dw - Trl(0, 0) * sin(d)),
          (standupPos(2) + dh + Trl(0, 1))
         };
    FR = {(standupPos(0) + Trr(1, 0) * cos(d+PI)),
          (standupPos(1) + dw - Trr(1, 0) * sin(d+PI)),
          (standupPos(2) + dh + Trr(1, 1))
         };
    RL = {(standupPos(0) - Trl(3, 0) * cos(d + 0.5*PI)),
          (standupPos(1) + dw - Trl(3, 0) * sin(d + 0.5*PI)),
          (standupPos(2) + dh + Trl(3, 1))
         };
    RR = {(standupPos(0) - Trr(2, 0) * cos(d + 1.5 * PI)),
          (standupPos(1) + dw - Trr(2, 0) * sin(d + 1.5 * PI)),
          (standupPos(2) + dh + Trr(2, 1))
         };
    //    Serial << "FL: " << FL << '\n';
    fl = Correction4Robot(FL);
    fr = Correction4Robot(FR);
    rl = Correction4Robot(RL);
    rr = Correction4Robot(RR);
    transitionPos(20, fl(0), fl(1), fl(2), fr(0), fr(1), fr(2), rl(0), rl(1), rl(2), rr(0), rr(1), rr(2));
  }
  if (state != 5)
  {
    Serial.println("Turn End!");
    threads.yield();
  }
}
// Bounce debouncer2 = Bounce();
Bounce debouncer4 = Bounce();
void setup()
{
  Serial.begin(1000000); // 设置串口波特率，与发送端匹配
  Serial2.begin(1000000);
  Serial3.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  //  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), onInterrupt2, FALLING);
  debouncer4.attach(4, INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  debouncer4.interval(5);
  attachInterrupt(digitalPinToInterrupt(3), onInterrupt3, FALLING);
  attachInterrupt(digitalPinToInterrupt(5), onInterrupt5, FALLING);

  //  while (!Serial)
  //  {
  //    delay(50);
  //  }
  digitalWrite(13, HIGH);
  delay(2000);
  threads.addThread(autoControlCommand);
  delay(5000);
  standup(90);
  threads.delay(3000);
  // sitdown(90);
  // threads.delay(3000);
  // threads.addThread(trotPlanner);
}

void onInterrupt2()
{
  if (isUp == 1)
  {
    Serial.println("Trotting");
    // sitdown();
    threads.addThread(trotPlanner);
    changeState(2);
    lf = 0.00;
    rf = 0.01;
  }
}
static void onInterrupt3()
{
  Serial.println("Turning Left");
  changeState(5);
  d = PI * 0.75;
  threads.addThread(TurnPlanner);
}
void onInterrupt5()
{

  Serial.println("Turning Right");
  changeState(5);
  d = -PI * 0.25;
  threads.addThread(TurnPlanner);

}
static void changeDirection(float a)
{
  veloAngle = a;
}
void loop()
{
  debouncer4.update();
  if (debouncer4.fell()) {
    changeState(1);
    standup(90);
    Serial.println("Standing");
  }
  if (state == 0)
  {
    Serial.println("bark");
  }

  if (Serial3.available())
  {
    char c = Serial3.read(); // 读取一个字符
    if (c == '0')
    {
      Serial.println("Trot L");
      lf = 0.00;
      rf = 0.03;
    }
    else if (c == '1')
    {
      Serial.println("IR5");
      lf = 0.00;
      rf = 0.005;
    }
    else if (c == '2')
    {
      Serial.println("Trot R");
      lf = 0.02;
      rf = 0.00;
    }
  }


  threads.delay(1);
  // sendControlCommand("11", "+066"); //11ID的电机，正转，123°//BIAS：0-6
  // sendControlCommand("12", "+030"); //12ID的电机，正转，123°//bias:1-68
  // sendControlCommand("13", "+060");//13ID的电机，正转，123°//bias:0-103
  //   sendZeroPointCommand(1, 2, 3); // 假设电机ID分别为1, 2, 3设置为0点
  //   sendMovementModeCommand(1, 3); // 假设电机ID为1，1ANGLE2SPEED3CURRENT
}