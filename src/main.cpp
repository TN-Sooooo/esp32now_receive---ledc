#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

int Motor_PWM_NUM;

#define Servo_PIN 1
#define MAX_WIDTH 2500
#define MIN_WIDTH 500

Servo myservo;

/*
 * LEDC Chan to Group/Channel/Timer Mapping
 ** ledc: 0  => Group: 0, Channel: 0, Timer: 0
 ** ledc: 1  => Group: 0, Channel: 1, Timer: 0
 ** ledc: 2  => Group: 0, Channel: 2, Timer: 1
 ** ledc: 3  => Group: 0, Channel: 3, Timer: 1
 ** ledc: 4  => Group: 0, Channel: 4, Timer: 2
 ** ledc: 5  => Group: 0, Channel: 5, Timer: 2
 ** ledc: 6  => Group: 0, Channel: 6, Timer: 3
 ** ledc: 7  => Group: 0, Channel: 7, Timer: 3
 ** ledc: 8  => Group: 1, Channel: 0, Timer: 0
 ** ledc: 9  => Group: 1, Channel: 1, Timer: 0
 ** ledc: 10 => Group: 1, Channel: 2, Timer: 1
 ** ledc: 11 => Group: 1, Channel: 3, Timer: 1
 ** ledc: 12 => Group: 1, Channel: 4, Timer: 2
 ** ledc: 13 => Group: 1, Channel: 5, Timer: 2
 ** ledc: 14 => Group: 1, Channel: 6, Timer: 3
 ** ledc: 15 => Group: 1, Channel: 7, Timer: 3
 */

// 绑定的IO

const int Motor_PWM_PinA = 4;

const int Motor_IN1 = 0;

const int Motor_IN2 = 3;

extern const int Motor_channel_PWMA = 2;
// PWM的通道，共16个(0-15)，分为高低速两组，
// 高速通道(0-7): 80MHz时钟，低速通道(8-15): 1MHz时钟
// 0-15都可以设置，只要不重复即可，参考上面的列表
// 如果有定时器的使用，千万要避开!!!

int Motor_freq_PWM = 1000; // 100khz
// PWM频率，直接设置即可

int Motor_resolution_PWM = 8;
// PWM分辨率，取值为 0-20 之间，这里填写为10，那么后面的ledcWrite
// 这个里面填写的pwm值就在 0 - 2的10次方 之间 也就是 0-1024

void Motor_Init(void)
{
  pinMode(Motor_IN1, OUTPUT);

  pinMode(Motor_IN2, OUTPUT);

  digitalWrite(Motor_IN1, LOW);

  digitalWrite(Motor_IN2, LOW);

  pinMode(Motor_PWM_PinA, OUTPUT);

  ledcSetup(Motor_channel_PWMA, Motor_freq_PWM, Motor_resolution_PWM); // 设置通道

  ledcAttachPin(Motor_PWM_PinA, Motor_channel_PWMA); // 将 LEDC 通道绑定到指定 IO 口上以实现输出
}

void PWM_SetDuty(uint16_t DutyA) // 设置占空比
{
  ledcWrite(Motor_channel_PWMA, DutyA);
}

int ADC_DATA;

// 接收数据的结构示例
// 在C中使用 typedef struct 定义一个结构体类型,名为struct_message
// 必须与发送方的结构相匹配一致

typedef struct struct_message
{
  int ADC_DATA;
  int servo_pwm;

} struct_message;
// 创建 结构为struct_message的myData变量
struct_message ADC_Data;

// 当收到数据时将执行的回调函数
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&ADC_Data, incomingData, sizeof(ADC_Data));
}

void setup()
{
  // 设置串口波特率
  Serial.begin(115200);

  // 设置设备WIFI模式为WIFI_STA
  WiFi.mode(WIFI_STA);

  Motor_Init();

  ESP32PWM::allocateTimer(0);

  myservo.setPeriodHertz(50);

  myservo.attach(Servo_PIN, MIN_WIDTH, MAX_WIDTH);

  // 初始化ESPNOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // 当ESPNOW初始化成功,我们将会注册一个回调函数（callback，CB）
  // 获得回收的包装信息
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{

  ADC_DATA = map(ADC_Data.ADC_DATA, 0, 1024, 255, 0);

  myservo.write(ADC_Data.servo_pwm);
  delay(2);

  if (ADC_DATA > 122)
  {
    Motor_PWM_NUM = map(ADC_DATA, 126, 255, 0, 255);
    // Serial.print("上半  :   ");
    // Serial.println(Motor_PWM_NUM);
    digitalWrite(Motor_IN1, LOW);
    digitalWrite(Motor_IN2, HIGH);
    ledcWrite(Motor_channel_PWMA, Motor_PWM_NUM);
  }
  else if (ADC_DATA < 110)
  {
    Motor_PWM_NUM = map(ADC_DATA, 0, 114, 255, 0);
     Serial.print("下半  :   ");
     Serial.println(Motor_PWM_NUM);
    digitalWrite(Motor_IN1, HIGH);
    digitalWrite(Motor_IN2, LOW);
    ledcWrite(Motor_channel_PWMA, Motor_PWM_NUM);
  }
  else
  {

    digitalWrite(Motor_IN1, LOW);
    digitalWrite(Motor_IN2, LOW);
     digitalWrite(Motor_PWM_PinA, LOW);
     Serial.print("ADC_DATA  :   ");
     Serial.println(ADC_DATA);
  }
}
