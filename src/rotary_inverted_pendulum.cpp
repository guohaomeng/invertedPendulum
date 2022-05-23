#include <SimpleFOC.h>
#include "pid_inc.hpp"
#include "Command.hpp"
//#include <Ticker.h>
#include "mywebsocket/mywebsocket.h"
#include <WiFi.h>

#define BOOL_TO_STR(bool_expr) (bool_expr) ? "true" : "false"

// BLDC 电机 & 驱动实例
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

// 编码器实例，使用AS5600，IIC通信
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// 内置电流传感器实例
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, 39, 36);

// 指令通信实例
// Commander command = Commander(Serial);
// void doMotor(char *cmd) { command.motor(&motor, cmd); }
// 简化版
Command command;

// 建立http及websocket服务器
myWebSocket::CombinedServer server;

IPAddress APIP = IPAddress(192, 168, 8, 1);
IPAddress subnet = IPAddress(255, 255, 255, 0);
bool websocket_init();

// 倒立摆初始角度：4.3° 目标角度：184.8°
float pendulum_angle_zero = 0.175;             // 倒立摆竖直向下时的角度 [rad]
float pendulum_angle = 4.559;                  // 倒立摆实时角度 [rad]
float angular_velocity = 0;                    // 摆杆角速度  [rad/s]
float target_angle = PI + pendulum_angle_zero; // 倒立摆摆杆目标角度 [rad]
float error;                                   // 摆杆角度误差角度 [rad]
float motor_velocity = 0;                      // 电机速度
float target_velocity = 0;                     // 倒立摆目标旋转速度,顺时针旋转为正
int index1 = 0;                                // 计数值
bool start_flag = false;                       // 倒立摆启动标志位
float swing_up_k = 0.7;                        // 能量法起摆参数
float theta, costheta;                         // 摆杆角度
LowPassFilter filter = LowPassFilter(0.005);
LowPassFilter base_filter = LowPassFilter(0.02);

// 定义倒立摆物理参数
float m1 = 0.01;                   // 旋转臂质量  [kg]
float l1 = 0.055;                  // 摆杆离圆心长度[m]
float m2 = 0.02;                   // 摆杆质量 [kg]
float l2 = 0.06;                   // 摆杆长度 [m]
float bar_j = (m2 * l2 * l2) / 12; // 摆杆相对质心的转动惯量 [kg/m^2]
float pendulum_energy = 0;         // 倒立摆系统能量
float E_0 = 0.0;
float angular_velocity_rad = angular_velocity * PI / 180.0f;
float torque_limit = 0.4;

float balance(float err_angle, float limit);
void control_loop(void);
void command_loop(void);
// 一个小状态机，包含了倒立摆的四个状态
enum pendulumState
{
  idle,      // 空闲状态
  swing_up,  // 能量法起摆状态过程
  control,   // 串级PID稳定立摆过程
  balanced,  // 摆已经稳定的过程
  swing_down // 落摆过程
};
pendulumState state = idle;

/**
 * 实例化一个摆杆角度环增量式PID控制器对象
 * 输入量：摆杆角度 pendulum_angle
 * 输出量：云台电机力矩分量1 output_torque1
 */

PID_INC PID_Angle = PID_INC(4.0f, 0.01f, 15.5f);
/**
 * 实例化一个旋转平台速度环增量式PID控制器对象
 * 输入量：电机当前速度 motor.shaft_velocity
 * 输出量：云台电机力矩分量2 output_torque2
 */

PID_INC PID_velocity = PID_INC(0.15f, 0.001, 0);

float dead_torque = 0.035; // 电机的死区摩擦力
float output_torque = 0.0; // 串级PID输出的最终力矩
float torque_correct(float t);

void do_ap(char *cmd) { command.scalar(&PID_Angle.P, cmd); }
void do_ai(char *cmd) { command.scalar(&PID_Angle.I, cmd); }
void do_ad(char *cmd) { command.scalar(&PID_Angle.D, cmd); }
void do_vp(char *cmd) { command.scalar(&PID_velocity.P, cmd); }
void do_vi(char *cmd) { command.scalar(&PID_velocity.I, cmd); }
void do_vd(char *cmd) { command.scalar(&PID_velocity.D, cmd); }

void timer_5ms() // 计算pid
{
  // 更新pid
  error = pendulum_angle - target_angle; // 误差值 = 当前值 - 目标值 [rad]

  float error_v = motor_velocity - target_velocity; // 电机速度，顺负逆正 力矩方向顺负逆正
  if (state != control)
  {
    return;
  }
  // pid运算
  float torque1 = PID_Angle.PID_Increase(error);
  float torque2 = PID_velocity.PID_Increase(error_v);
  output_torque = motor.target + torque1 + torque2;
  Serial.printf("t:%.3f,%.3f,%.3f,%.3f\n", pendulum_angle, target_angle, motor_velocity, output_torque);
  if (output_torque > torque_limit)
  {
    output_torque = torque_limit;
  }
  if (output_torque < -torque_limit)
  {
    output_torque = -torque_limit;
  }
  // output_torque = -output_torque;
}

void angleTask2(void *arg)
{
  Serial.printf("timer1&2 ok\n");
  // 初始化websockets及http服务器
  websocket_init();
  Serial.printf("websocket ok\n");
  while (true)
  {
    // unsigned long time1 = micros();
    server.loop();
    if (!start_flag)
    {
      Serial.printf("a:%.3f,%.3f,%.3f\n", pendulum_angle, angular_velocity, output_torque);
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
    // Serial.printf("time:%d\n",micros()-time1);
  }
  vTaskDelete(NULL);
}

void setup()
{

  I2Cone.begin(19, 18, 400000);
  I2Ctwo.begin(23, 5, 400000);
  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  motor.linkSensor(&sensor0);
  // 驱动器配置
  // 电源供电电压 [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // 连接驱动
  motor.linkDriver(&driver);
  // 电流限制 [A]
  motor.current_limit = 0.4;
  // 电压限制 [V]
  motor.voltage_limit = 12;
  // FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // TorqueControlType::foc_current //力矩控制模式，默认使用foc电流
  // 其他模式 TorqueControlType::voltage TorqueControlType::dc_current
  motor.torque_controller = TorqueControlType::foc_current;
  // 控制环模式，默认为力矩控制模式
  motor.controller = MotionControlType::torque;

  // d轴电流pid设置
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 300;
  motor.PID_current_d.D = 0;
  motor.LPF_current_d.Tf = 0.01;
  // q轴电流pid设置
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 300;
  motor.PID_current_q.D = 0;
  motor.LPF_current_q.Tf = 0.01;

  // 速度低通滤波时间常数 1/(Tf*s +1)
  motor.LPF_velocity.Tf = 0.1;
  // 角度环速度限制 [rad/s]
  motor.velocity_limit = 20;

  // 对电机初始化使用串行监控
  // 监控端口
  Serial.begin(115200);
  // 不需要请注释掉
  // motor.useMonitoring(Serial);                                   // 启用串口监控
  // motor.monitor_downsample = 0;                                  // 默认禁用
  // motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // 监控目标速度和角度

  //电流传感器初始化和链接
  current_sense.init();
  current_sense.gain_b *= -1;
  motor.linkCurrentSense(&current_sense);

  //初始化电机
  motor.init();
  // 对齐编码器并启动 FOC
  motor.initFOC();

  // 设置初始目标值
  motor.target = 0;

  // 增加M指令
  command.add("AP", do_ap);
  command.add("AI", do_ai);
  command.add("AD", do_ad);
  command.add("VP", do_vp);
  command.add("VI", do_vi);
  command.add("VD", do_vd);

  // 启动web服务器任务
  //xTaskCreatePinnedToCore(angleTask2, "angleTask2", 24 * 4096, NULL, 1, NULL, 0);
  Serial.printf("任务2初始化成功\n");

  vTaskDelay(3000 / portTICK_PERIOD_MS);
}

void loop()
{
  sensor0.update(); // 更新电机角度值 240us
  sensor1.update(); // 更新摆杆角度值 240us
  pendulum_angle = sensor1.getMechanicalAngle();
  angular_velocity = filter(sensor1.getVelocity());
  motor_velocity = base_filter(motor.shaft_velocity);
  control_loop();
  // 迭代设置 FOC 相电压
  motor.loopFOC();
  // 力矩矫正限幅等
  output_torque = torque_correct(output_torque);
  // 设置出路环目标的迭代函数
  motor.move(output_torque);
  //电机监控
  // motor.monitor();

  // 用户通信
  command_loop();
}

void control_loop(void)
{
  // 更新摆杆角度值
  error = pendulum_angle - target_angle; // 误差值 = 当前值 - 目标值 [rad]

  if (!start_flag) // 未启动倒立摆控制程序则返回
  {
    output_torque = 0;
    state = idle;
    index1 = 0;
    return;
  }

  switch (state)
  {
  case idle: // idle 空状态，当摆竖直朝下的时候，给一个脉冲，不然energy shaping法无法启动，摆不能自己摆起来
    if (index1 < 100)
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
      output_torque = 0.15;
      if (index1 > 50)
      {
        output_torque = -output_torque;
      }
      index1++;
    }
    else
      state = swing_up;
    break;
  case swing_up: // swing up 使用energy shaping法摆起来，接近平衡位置的时候切换状态到串级PID控制
    theta = 3.14159f + error;
    costheta = cos(theta);
    pendulum_energy = 0.5 * bar_j * angular_velocity * angular_velocity - m2 * 9.8 * l2 * costheta;
    output_torque = swing_up_k * (-angular_velocity) * costheta * (pendulum_energy - E_0); // 能量法起摆公式：u= u_k * (E - E_0) * d_theta * cos(theta)
    // Serial.printf("p/t:%f,%f,%.3f,%.3f\n",pendulum_energy,output_torque,angular_velocity,theta);
    if (abs(error) < 0.25) // 小于0.45进入控制
    {
      output_torque = 0;
      state = control;
    }
    break;
  case control:
    timer_5ms();
    // output_torque = -output_torque;
    // Serial.printf("o:%.3f,%.3f\n", error, output_torque);
    if (abs(error) > 0.3)
      state = swing_down;
    break;
  case swing_down:
    output_torque = 0;
    // Serial.printf("t:%.3f,%.3f\n",abs(pendulum_angle - pendulum_angle_zero),abs(angular_velocity_rad));
    if (abs(pendulum_angle - pendulum_angle_zero) < 0.05 && (abs(angular_velocity) < 1))
    {
      state = idle;
      index1 = 0;
    }
    break;
  default:
    output_torque = 0;
    break;
  }
}

void command_loop(void)
{
  // 如果串口是空的直接返回
  if (Serial.available() == 0)
    return;
  char received_chars[10];
  memset(received_chars, '\0', 10);
  vTaskDelay(1 / portTICK_PERIOD_MS);
  // 从串口读取返回的数据，读取20个字符
  Serial.read(received_chars, 10);

  // 根据指令做不同动作
  if (received_chars[0] == 'A' && received_chars[1] == 'P')
    do_ap(received_chars + 2);
  if (received_chars[0] == 'A' && received_chars[1] == 'I')
    do_ai(received_chars + 2);
  if (received_chars[0] == 'A' && received_chars[1] == 'D')
    do_ad(received_chars + 2);
  if (received_chars[0] == 'V' && received_chars[1] == 'P')
    do_vp(received_chars + 2);
  if (received_chars[0] == 'V' && received_chars[1] == 'I')
    do_vi(received_chars + 2);
  if (received_chars[0] == 'V' && received_chars[1] == 'D')
    do_vd(received_chars + 2);
  if (received_chars[0] == 'E' && received_chars[1] == 'N')
  {
    start_flag = !start_flag;
    Serial.printf("EN:%s\n", BOOL_TO_STR(start_flag));
  }

  if (received_chars[0] == 'T') // T指令设置力矩
  {
    command.scalar(&output_torque, received_chars + 1);
    Serial.printf("%s,%.3f\n", received_chars, output_torque);
  }
  if (received_chars[0] == 'K') // K指令设置swing_up_k
  {
    command.scalar(&swing_up_k, received_chars + 1);
    Serial.printf("%s,%f\n", received_chars, swing_up_k);
  }
  if (received_chars[0] == 'E' && received_chars[1] != 'N') // E指令设置E_0
  {
    command.scalar(&E_0, received_chars + 1);
    Serial.printf("%s,%f\n", received_chars, E_0);
  }
  if (received_chars[0] == 'S') // T指令设置力矩
  {
    command.scalar(&target_velocity, received_chars + 1);
    Serial.printf("%s,%.3f\n", received_chars, target_velocity);
  }
  //  最后清空串口
  while (Serial.read() >= 0)
    ;
}

float torque_correct(float t)
{
  // 消除死区摩擦力影响
  // if (t > 0 && t < dead_torque)
  //   t += dead_torque;
  // if (t < 0 && t > -dead_torque)
  //   t -= dead_torque;
  // 力矩限幅
  if (t > torque_limit)
    t = torque_limit;
  if (t < -torque_limit)
    t = -torque_limit;
  return t;
}
// 服务器初始化
bool websocket_init()
{
  //调用函数启用ESP32硬件加速
  mycrypto::SHA::initialize();
  //Serial.println("Connecting to WiFi...");
  // WiFi.begin("meng", "12345678");
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   yield();
  //   vTaskDelay(100 / portTICK_PERIOD_MS);
  //   yield();
  // }
  //Serial.println("WiFi connected.");
  WiFi.softAP("ESP32_WebSocketServer");
  vTaskDelay(300 / portTICK_PERIOD_MS);
  WiFi.softAPConfig(APIP, APIP, subnet);
  // 设置websocket服务器及回调函数
  server.setCallback(
      [](myWebSocket::WebSocketClient *client, myWebSocket::WebSocketEvents type, uint8_t *payload, uint64_t length)
      {
        if (length)
        {
          if (type == myWebSocket::TYPE_TEXT)
          {
            Serial.println("Got text data:");
            Serial.println(String((char *)payload));
            client->send(String("Hello from ESP32 WebSocket: ") + String(ESP.getFreeHeap()));
          }
          else if (type == myWebSocket::TYPE_BIN)
          {
            Serial.println("Got binary data, length: " + String((long)length));
            Serial.println("First byte: " + String(payload[0]));
            Serial.println("Last byte: " + String(payload[length - 1]));
          }
          else if (type == myWebSocket::WS_DISCONNECTED)
          {
            Serial.println("Websocket disconnected.");
          }
          else if (type == myWebSocket::WS_CONNECTED)
          {
            Serial.println("Websocket connected.");
          }
        }
      });
  // 开启网页服务器
  server.on(
      "/",
      [](myWebSocket::ExtendedWiFiClient *client, myWebSocket::HttpMethod method, uint8_t *data, uint64_t len)
      {
        client->send(R"(
<!DOCTYPE html>
<html>

<head>
  <title>ESP32 Combined Server</title>
</head>

<body>
  <h1>Hello World!</h1>

</body>

</html>
)");
        client->close();
      });
  server.begin(80);
  return true;
}