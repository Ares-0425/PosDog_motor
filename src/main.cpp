#include <Arduino.h>
#include <SimpleFOC.h>
//#include <CAN.h>
#include "can.hpp"
#include "dob.hpp"

#define LEG 4  // 1:右前脚, 2:左前脚, 3:右後脚, 4:左後脚

// # SimpleFOC関連
// BLDC motor & driver instance
#define INPUT_VOLTAGE 12 // 電源電圧 [V]

BLDCMotor motor = BLDCMotor(14);
BLDCDriver6PWM driver = BLDCDriver6PWM(26, 25, 27, 33, 14, 32);

// encoder instance
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, SS);

//  LowsideCurrentSense(mVpA, adc_a, adc_b, adc_c)
#if (LEG == 1)
  LowsideCurrentSense current_sense = LowsideCurrentSense(500.0, 34, 39, 36);
#elif (LEG == 2)
  LowsideCurrentSense current_sense = LowsideCurrentSense(500.0, 39, 34, 36);
#elif (LEG == 3)
  LowsideCurrentSense current_sense = LowsideCurrentSense(500.0, 39, 34, 36);
#else
  LowsideCurrentSense current_sense = LowsideCurrentSense(500.0, 36, 34, 39);
#endif

// instantiate the commander
Commander command = Commander(Serial,'\n',true);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void SimpleFOC_setup();

// コントロールモード
#define ANGLEMODE 0
#define VELOCITYMODE 1
#define TORQUEMODE 2
uint8_t control_mode = ANGLEMODE;
boolean is_moving = false; // モーターが動いているかどうか

// 他
float sens_cash = 0.0;  // センサー値の一時保存
float control_cash = 0.0; // コントロール値の一時保存
float control_torque = 0.0;
float torque_cash = 0.0;
float torque_est = 0.0;
float target_torque = 0.0;
float current_limit = 0.5;
float omega;
float alpha;
float current_cash = 0.0; // 電流値の一時保存
float leg_angle;  // 脚の角度(rad)

float sensor_offset = 0.0;
float angle_limit = radians(90.0); // 角度制限(rad)

unsigned long time_now,time_old;
float dt;

#define photo_Pin 35
boolean photo_enable = false; // 光センサーの有効化フラグ
boolean photo_state = false; // 光センサーの状態
boolean photo_old = false; // 光センサーの前回の状態

#define state_Pin 17 // 状態表示用LEDピン

// const int gear_rate = 16;       //ギア比(dobで定義済み)

#if (LEG == 1)
  // 自分のCANID
  #define CAN_ID 0x201
  // 受信するCANID
  #define CAN_MASK 0x101
  // センサーゼロ点
  const float zero_angle = 3.22;
#elif (LEG == 2)
  #define CAN_ID 0x202
  #define CAN_MASK 0x102
  const float zero_angle = 3.44;
#elif (LEG == 3)
  #define CAN_ID 0x203
  #define CAN_MASK 0x103
  const float zero_angle = 0.49;
#else
  #define CAN_ID 0x204
  #define CAN_MASK 0x104
  const float zero_angle = 0.49;
#endif

KalmanEstimator kalman;

TaskHandle_t focTaskHandle = NULL;  // タスクハンドル
void focTask(void *pvParameters);

void CAN_read();

void calibration();
void calibration_2();

void setup() {
  Serial.begin(115200); // シリアル通信速度の設定
  pinMode(photo_Pin, INPUT);
  pinMode(state_Pin, OUTPUT);
  digitalWrite(state_Pin, HIGH);
  
  SimpleFOC_setup();  // SimpleFOCの設定
  motor.disable();
  driver.disable();

  kalman.init();

  CAN_setup(CAN_ID, CAN_MASK);  // CANの設定
  send_CANdata[0] = 'R';
  send_length = 1;
  send_flag = true;

  // Core1にFOC loopタスクを作成
  xTaskCreatePinnedToCore(
    focTask,   // タスク関数
    "FOC",         // タスク名
    4096,             // スタックサイズ
    NULL,             // 引数
    1,                // 優先度
    &focTaskHandle,   // タスクハンドル
    1                 // Core1を指定
  );

  digitalWrite(state_Pin, LOW);
}

int t_count = 0;
void loop() {
  // main FOC algorithm function
  if(receive_flag) {
    // CANメッセージを受信した場合の処理
    CAN_read(); // CANメッセージの処理
    receive_flag = false; // フラグをリセット
  }

  time_old = time_now;
  time_now = micros();
  dt = (time_now - time_old) * 1e-6f;

  // motor.loopFOC();
  // sensor.update();

  kalman.update(sensor.getAngle()/gear_rate, dt);

  float omega = kalman.getOmega();
  float alpha = kalman.getAlpha();

  leg_angle = (sensor.getAngle() - sensor_offset) / gear_rate;

  if(is_moving){
    if(control_mode == ANGLEMODE){
      torque_est = Disturbance_OB(motor.current.q*KT*gear_rate, alpha, omega, leg_angle);
      control_torque = PID_position(control_cash, leg_angle, omega, dt);

      target_torque = (control_torque + torque_est) / (KT * gear_rate);
      // if(target_torque > torque_limit/gear_rate/KT) target_torque = torque_limit/gear_rate/KT;
      // if(target_torque < -torque_limit/gear_rate/KT) target_torque = -torque_limit/gear_rate/KT;
      if(target_torque > current_limit) target_torque = current_limit;
      if(target_torque < -current_limit) target_torque = -current_limit;
      motor.target = target_torque;

      // t_count++;
      // if(t_count > 10){
      //   t_count = 0;
      //   Serial.printf("P:%f %f\n",target_torque,angle);
      // }
    }else if(control_mode == VELOCITYMODE){
      // control_torque = PID_velocity(control_cash, omega, dt);

      // target_torque = control_torque / KT / gear_rate;
      // motor.target = target_torque;
      // Serial.printf("S:%d %f\n",control_torque, dt);
    }
    if(leg_angle > angle_limit){
      torque_est = Disturbance_OB(target_torque*KT*gear_rate, alpha, omega, leg_angle);
      control_torque = PID_position(angle_limit-0.01, leg_angle, omega, dt);

      target_torque = (control_torque + torque_est) / (KT * gear_rate);
      if(target_torque > 2.4) target_torque = 2.4;
      if(target_torque < -2.4) target_torque = -2.4;
      motor.target = target_torque;
    }else if(leg_angle < -angle_limit){
      torque_est = Disturbance_OB(target_torque*KT*gear_rate, alpha, omega, leg_angle);
      control_torque = PID_position(-angle_limit+0.01, leg_angle, omega, dt);

      target_torque = (control_torque + torque_est) / (KT * gear_rate);
      if(target_torque > 2.4) target_torque = 2.4;
      if(target_torque < -2.4) target_torque = -2.4;
      motor.target = target_torque;
    }
  }

  // motor.move(target_torque);
}

void SimpleFOC_setup() {
  // デバッグ出力を行う
  SimpleFOCDebug::enable(&Serial);
  
  // センサーの初期化
  sensor.init();
  // モータとセンサーをリンク
  motor.linkSensor(&sensor);
  // set calibration values
  motor.zero_electric_angle  = zero_angle; // rad
  motor.sensor_direction = Direction::CW; // CW or CCW

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = INPUT_VOLTAGE;
  // Max DC voltage allowed - default voltage_power_supply
  driver.enable_pin = 21; // nSLEEP pin
  driver.enable_active_high = true; // enable pin is active high
  driver.dead_zone = 0.02; // dead time for the pwm signal [0,1] - 2% of the pwm cycle
  // driver init
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // init current sense
  current_sense.init();
  // link the driver with the current sense
  current_sense.linkDriver(&driver);

  // velocity loop PID
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 10.0;
  motor.PID_velocity.limit = 0.5;
  // Low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.05;
  // angle loop PID
  motor.P_angle.P = 20.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = 70.0;
  // Low pass filtering time constant 
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID 
  motor.PID_current_q.P = 30.0; //30.0
  motor.PID_current_q.I = 100.0;  //100.0
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.output_ramp = 0.0; // 0.0
  motor.PID_current_q.limit = 12.0;
  // Low pass filtering time constant 
  motor.LPF_current_q.Tf = 0.05;
  // current d loop PID
  motor.PID_current_d.P = 30.0;
  motor.PID_current_d.I = 100.0;
  motor.PID_current_d.D = 0.0;
  motor.PID_current_d.output_ramp = 0.0;
  motor.PID_current_d.limit = 12.0;
  // Low pass filtering time constant 
  motor.LPF_current_d.Tf = 0.05;
  // Limits 
  motor.velocity_limit = 70.0;
  motor.voltage_limit = 12.0;
  motor.current_limit = 0.81;

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();

  // init current sense
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  // invert phase b gain
  #if (LEG == 1)

  #elif (LEG == 4)
    current_sense.gain_a *=-1;
    current_sense.gain_b *=-1;
    current_sense.gain_c *=-1;
  #else
    current_sense.gain_a *=-1;
    current_sense.gain_b *=-1;
  #endif
  //current_sense.gain_c *=-1;
  // skip alignment
  current_sense.skip_align = true;

  // start the FOC
  motor.initFOC();

  // add target command M
  //command.add('M', doMotor, "Motor");
  // tell the motor to use the monitoring
  //motor.useMonitoring(Serial);
  //motor.monitor_downsample = 0; // disable monitor at first - optional
}

void focTask(void *pvParameters) {
  while (true) {
    motor.loopFOC();
    //sensor.update();
    
    motor.move();
  }
}

void CAN_read() {
  //if (receive_id == CAN_MASK){
    switch(receive_CANdata[0]){
      case 'A': // モーターの角度を取得
        //sensor.update();
        current_cash = current_sense.getDCCurrent();
        // send_CANdata[0] = 'A';
        // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降にfloatをコピー
        // send_length = 1 + sizeof(float); // 5
        ((float*)&send_CANdata[0])[0] = leg_angle; // 0バイト目以降にfloatをコピー
        ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        send_length = sizeof(float) + sizeof(float); // 8
        send_flag = true; // 送信フラグを立てる
        break;
      case 'V': // モーターの速度を取得
        //sensor.update();
        sens_cash = sensor.getVelocity();
        current_cash = current_sense.getDCCurrent();
        // send_CANdata[0] = 'V';
        // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降にfloatをコピー
        // send_length = 1 + sizeof(float); // 5
        ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
        ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        send_length = sizeof(float) + sizeof(float); // 8
        send_flag = true; // 送信フラグを立てる
        break;
      case 'C': // モーターの電流を取得
        break;
      case 'T': // モーターのトルクを指定
        if(control_mode != TORQUEMODE) control_mode = TORQUEMODE;
        control_cash = *((float*)&receive_CANdata[1]); // 1バイト目以降をfloatとして取得
        motor.target = control_cash; // 目標トルクを設定

        //current_sense.update();
        current_cash = current_sense.getDCCurrent(); // 電流を取得
        // send_CANdata[0] = 'A'; // 角度取得のコマンド
        // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に角度をfloatとしてコピー
        // send_length = 1 + sizeof(float); // 5
        ((float*)&send_CANdata[0])[0] = leg_angle; // 0バイト目以降にfloatをコピー
        ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        send_length = sizeof(float) + sizeof(float); // 8
        send_flag = true; // 送信フラグを立てる
        // if(receive_length == 6){
        //   if(receive_CANdata[5] == 'A'){
        //     sensor.update();
        //     sens_cash = sensor.getAngle(); // 角度を取得
        //     current_cash = current_sense.getDCCurrent(); // 電流を取得
        //     // send_CANdata[0] = 'A'; // 角度取得のコマンド
        //     // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に角度をfloatとしてコピー
        //     // send_length = 1 + sizeof(float); // 5
        //     ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
        //     ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        //     send_length = sizeof(float) + sizeof(float); // 8
        //     send_flag = true; // 送信フラグを立てる
        //   }else if(receive_CANdata[5] == 'V'){
        //     sensor.update();
        //     sens_cash = sensor.getVelocity(); // 速度を取得
        //     current_cash = current_sense.getDCCurrent(); // 電流を取得
        //     // send_CANdata[0] = 'V'; // 速度取得のコマンド
        //     // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に速度をfloatとしてコピー
        //     // send_length = 1 + sizeof(float); // 5
        //     ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
        //     ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        //     send_length = sizeof(float) + sizeof(float); // 8
        //     send_flag = true; // 送信フラグを立てる
        //   }
        // }
        break;
      case 'S': // モーターの速度を指定
        if(control_mode == VELOCITYMODE){
          control_cash = *((float*)&receive_CANdata[1]); // 1バイト目以降をfloatとして取得
          motor.target = control_cash; // 目標速度を設定

          //current_sense.update();
          current_cash = current_sense.getDCCurrent(); // 電流を取得
          // send_CANdata[0] = 'A'; // 角度取得のコマンド
          // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に角度をfloatとしてコピー
          // send_length = 1 + sizeof(float); // 5
          ((float*)&send_CANdata[0])[0] = leg_angle; // 0バイト目以降にfloatをコピー
          ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
          send_length = sizeof(float) + sizeof(float); // 8
          send_flag = true; // 送信フラグを立てる
        }else if(control_mode == ANGLEMODE){
          //control_cash = *((float*)&receive_CANdata[1]); // 1バイト目以降をfloatとして取得
          //motor.velocity_limit = control_cash; // 速度制限を設定
        }
        // if(receive_length == 6){
        //   if(receive_CANdata[5] == 'A'){
        //     sensor.update();
        //     sens_cash = sensor.getAngle(); // 角度を取得
        //     current_cash = current_sense.getDCCurrent(); // 電流を取得
        //     // send_CANdata[0] = 'A'; // 角度取得のコマンド
        //     // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に角度をfloatとしてコピー
        //     // send_length = 1 + sizeof(float); // 5
        //     ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
        //     ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        //     send_length = sizeof(float) + sizeof(float); // 8
        //     send_flag = true; // 送信フラグを立てる
        //   }else if(receive_CANdata[5] == 'V'){
        //     sensor.update();
        //     sens_cash = sensor.getVelocity(); // 速度を取得
        //     current_cash = current_sense.getDCCurrent(); // 電流を取得
        //     // send_CANdata[0] = 'V'; // 速度取得のコマンド
        //     // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に速度をfloatとしてコピー
        //     // send_length = 1 + sizeof(float); // 5
        //     ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
        //     ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        //     send_length = sizeof(float) + sizeof(float); // 8
        //     send_flag = true; // 送信フラグを立てる
        //   }
        // }
        break;
      case 'P': // モーターの位置を指定
        if(control_mode != ANGLEMODE) control_mode = ANGLEMODE;
        control_cash = *((float*)&receive_CANdata[1]); // 1バイト目以降をfloatとして取得
        //motor.target = control_cash; // 目標角度を設定

        //current_sense.update();
        current_cash = current_sense.getDCCurrent(); // 電流を取得
        // send_CANdata[0] = 'A'; // 角度取得のコマンド
        // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に角度をfloatとしてコピー
        // send_length = 1 + sizeof(float); // 5
        ((float*)&send_CANdata[0])[0] = leg_angle; // 0バイト目以降にfloatをコピー
        ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        send_length = sizeof(float) + sizeof(float); // 8
        send_flag = true; // 送信フラグを立てる
        // if(receive_length == 6){
        //   if(receive_CANdata[5] == 'A'){
        //     sensor.update();
        //     sens_cash = sensor.getAngle(); // 角度を取得
        //     current_cash = current_sense.getDCCurrent(); // 電流を取得
        //     // send_CANdata[0] = 'A'; // 角度取得のコマンド
        //     // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に角度をfloatとしてコピー
        //     // send_length = 1 + sizeof(float); // 5
        //     ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
        //     ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        //     send_length = sizeof(float) + sizeof(float); // 8
        //     send_flag = true; // 送信フラグを立てる
        //   }else if(receive_CANdata[5] == 'V'){
        //     sensor.update();
        //     sens_cash = sensor.getVelocity(); // 速度を取得
        //     current_cash = current_sense.getDCCurrent(); // 電流を取得
        //     // send_CANdata[0] = 'V'; // 速度取得のコマンド
        //     // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に速度をfloatとしてコピー
        //     // send_length = 1 + sizeof(float); // 5
        //     ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
        //     ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
        //     send_length = sizeof(float) + sizeof(float); // 8
        //     send_flag = true; // 送信フラグを立てる
        //   }
        //}
        break;
      case 'E': // モーターのONOFF
        if(receive_CANdata[1] == '1') {
          is_moving = true;
          digitalWrite(state_Pin, HIGH);
          motor.PID_velocity.reset();
          motor.P_angle.reset();
          motor.PID_current_q.reset();
          motor.PID_current_d.reset();
          driver.enable(); // ドライバを有効化
          motor.enable(); // モーターを有効化
        } else if (receive_CANdata[1] == '0') {
          motor.disable(); // モーターを無効化
          driver.disable(); // ドライバを無効化
          is_moving = false;
          digitalWrite(state_Pin, LOW);
          motor.PID_velocity.reset();
          motor.P_angle.reset();
          motor.PID_current_q.reset();
          motor.PID_current_d.reset();
        }
        break;
      case 'L': // モーターのキャリブレーション
        calibration();
        break;
      case 'M': // モーターのモード変更
        motor.disable(); // モーターを一旦停止
        driver.disable(); // ドライバを一旦停止
        if(receive_CANdata[1] == 'P') {
          //motor.controller = MotionControlType::angle;
          motor.controller = MotionControlType::torque;
          control_cash = *((float*)&receive_CANdata[2]);
          //motor.target = control_cash;
          control_mode = ANGLEMODE; // 角度制御モード
        } else if (receive_CANdata[1] == 'S') {
          motor.controller = MotionControlType::velocity;
          control_cash = *((float*)&receive_CANdata[2]);
          motor.target = control_cash;
          control_mode = VELOCITYMODE; // 速度制御モード
        } else if (receive_CANdata[1] == 'T') {
          motor.controller = MotionControlType::torque;
          control_cash = *((float*)&receive_CANdata[2]);
          motor.target = control_cash;
          control_mode = TORQUEMODE; // トルク制御モード
        }
        if(is_moving == true) {
          driver.enable(); // ドライバを再開
          motor.enable(); // モーターを再開
        }
        if(receive_length == 7){
          if(receive_CANdata[6] == 'A'){
            sensor.update();
            sens_cash = sensor.getAngle(); // 角度を取得
            current_cash = current_sense.getDCCurrent(); // 電流を取得
            // send_CANdata[0] = 'A'; // 角度取得のコマンド
            // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に角度をfloatとしてコピー
            // send_length = 1 + sizeof(float); // 5
            ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
            ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
            send_length = sizeof(float) + sizeof(float); // 8
            send_flag = true; // 送信フラグを立てる
          }else if(receive_CANdata[6] == 'V'){
            sensor.update();
            sens_cash = sensor.getVelocity(); // 速度を取得
            current_cash = current_sense.getDCCurrent(); // 電流を取得
            // send_CANdata[0] = 'V'; // 速度取得のコマンド
            // ((float*)&send_CANdata[1])[0] = sens_cash; // 1バイト目以降に速度をfloatとしてコピー
            // send_length = 1 + sizeof(float); // 5
            ((float*)&send_CANdata[0])[0] = sens_cash; // 0バイト目以降にfloatをコピー
            ((float*)&send_CANdata[4])[0] = current_cash; // 4バイト目以降にfloatをコピー
            send_length = sizeof(float) + sizeof(float); // 8
            send_flag = true; // 送信フラグを立てる
          }
        }
        break;
      case 'B': // リミット設定
        if(receive_CANdata[1] == 'T'){ // トルク(電流)リミット設定
          motor.current_limit = *((float*)&receive_CANdata[2]);
          current_limit = *((float*)&receive_CANdata[2]);
        }else if(receive_CANdata[1] == 'V'){ // 速度リミット設定
          
        }else if(receive_CANdata[1] == 'A'){ // 角度リミット設定
          angle_limit = radians(*((float*)&receive_CANdata[2]));
        }
        break;
      case 'R': // 基板のリセット
        ESP.restart();
        break;
      default:
        break;
    }
  //}
}

void calibration(){
  /*
  1. まず位置を左側白の境界線に合わせる（angle_cash1)
  2. 反時計回りで左側黒の境界線に合わせる（angle_cash2)
  3. 反時計回りで反対側に移動する
  4. 時計回りで左側白の境界線に合わせる（angle_cash3)
  5. 時計回りで左側黒の境界線に合わせる（angle_cash4)
  6. 4点の平均をセンサーオフセットとする
  */
  float angle_cash1 = 0.0;
  float angle_cash2 = 0.0;
  float angle_cash3 = 0.0;
  float angle_cash4 = 0.0;
  int posision_num = 0;
  float angle_cash = 0.0;

  motor.controller = MotionControlType::velocity;
  control_mode = VELOCITYMODE;
  control_cash = 0.0;
  is_moving = true;
  digitalWrite(state_Pin, HIGH);
  driver.enable();
  motor.enable();

  time_now = millis();
  motor.target = -10.0; // 時計回り

  while(digitalRead(photo_Pin) == HIGH){  // 光センサーが反応するまでループ

    if(sensor.getAngle() < -0.5*gear_rate){  //30度以上回転したら
      motor.target = 10.0;
      while(digitalRead(photo_Pin) == HIGH){  // 光センサーが反応するまで待機
        delayMicroseconds(1000);

        if(sensor.getAngle() > 0.5*gear_rate){  //60度以上回転したら
          motor.target = 0.0;
          motor.disable();
          driver.disable();
          is_moving = false;
          digitalWrite(state_Pin, LOW);
          delay(1000);
          
          send_CANdata[0] = 'L';  //キャリブレーション失敗
          send_CANdata[1] = '0';
          send_length = 2;
          while(send_flag == true);
          send_flag = true;
          while(send_flag == true);
          delay(1000);

          ESP.restart();
        }
      }
      // 時計回りで見つからず、反時計回りで見つかった場合の処理(4)
      posision_num = 4;
      angle_cash = sensor.getAngle(); // 角度を取得
    }
  }
  // 時計回りで見つかった場合の処理
  while(digitalRead(photo_Pin) == LOW && posision_num == 0);
  angle_cash = sensor.getAngle(); // 角度を取得
  while(sensor.getAngle() > angle_cash - 0.20*gear_rate && posision_num == 0);  // 10度以上回転するまでループ
  if(posision_num == 0){
    posision_num = 5;
  }

  if(posision_num == 5){
    motor.target = 10.0;
    while(digitalRead(photo_Pin) == HIGH);
    posision_num = 4;
  }

  if(posision_num == 4){
    angle_cash1 = sensor.getAngle(); // 角度を取得
    motor.target = 10.0;
    while(digitalRead(photo_Pin) == LOW);
    angle_cash2 = sensor.getAngle(); // 角度を取得

    motor.target = 20.0; // 倍速
    while(sensor.getAngle() < angle_cash2 + 0.18*gear_rate);  // 10度以上回転するまでループ

    motor.target = -10.0;
    while(digitalRead(photo_Pin) == HIGH);
    angle_cash3 = sensor.getAngle();
    while(digitalRead(photo_Pin) == LOW);
    angle_cash4 = sensor.getAngle();

    motor.target = 0.0; // モーターを停止

    motor.disable();
    driver.disable();

    //motor.controller = MotionControlType::angle; // 角度制御モードに設定
    motor.controller = MotionControlType::torque;
    control_mode = ANGLEMODE;
    sensor_offset = (angle_cash1 + angle_cash2 + angle_cash3 + angle_cash4) / 4.0; // センサーオフセットを計算
    control_cash = 0.0;
    //motor.velocity_limit = 15.0;

    driver.enable();
    motor.enable();
  }
  send_CANdata[0] = 'L';  //キャリブレーション成功
  send_CANdata[1] = '1';
  send_length = 2;
  while(send_flag == true);
  send_flag = true;
}

void calibration_2(){
  /*
  1. まず位置を左側白の境界線に合わせる（angle_cash1)
  2. 反時計回りで左側黒の境界線に合わせる（angle_cash2)
  3. 反時計回りで反対側に移動する
  4. 時計回りで左側白の境界線に合わせる（angle_cash3)
  5. 時計回りで左側黒の境界線に合わせる（angle_cash4)
  6. 4点の平均をセンサーオフセットとする
  */
  float angle_cash = 0.0;
  float angle_cash1 = 0.0;
  float angle_cash2 = 0.0;
  float angle_cash3 = 0.0;
  float angle_cash4 = 0.0;
  float angle_cash5 = 0.0;
  float angle_cash6 = 0.0;
  float angle_cash7 = 0.0;
  float angle_cash8 = 0.0;
  int posision_num = 0;

  motor.controller = MotionControlType::velocity;
  control_mode = VELOCITYMODE;
  control_cash = 0.0;
  is_moving = true;
  digitalWrite(state_Pin, HIGH);
  driver.enable();
  motor.enable();

  time_now = millis();

  if(digitalRead(photo_Pin) == HIGH){ // 最初黒（2or4）
    motor.target = 10.0; // 反時計回り
    while(sensor.getAngle() < -radians(15.0)*gear_rate); // 15度以上回転するまでループ
    motor.target = 0.0;
    posision_num = 1;
  }

  while(sensor.getAngle() < radians(30.0)*gear_rate && posision_num != 1){
    motor.target = 10.0; // 反時計回り
    if(digitalRead(photo_Pin) == HIGH){ // 黒検出(3or5)
      angle_cash = sensor.getAngle();
      while(sensor.getAngle() < angle_cash + radians(15.0)*gear_rate); // 15度以上回転するまでループ
      motor.target = 0.0;
      posision_num = 1;
    }
  }
  if(posision_num == 0){ // もともと1だった
    motor.target = -10.0; // 時計回り
    while(sensor.getAngle() > radians(5.0)*gear_rate); // 元の角度+5度以下までループ
    motor.target = 0.0;
    posision_num = 1;
  }

  angle_cash = sensor.getAngle();

  motor.target = -10.0; // 時計回り
  while(digitalRead(photo_Pin) == LOW){  // 白の間ループ
    if(sensor.getAngle() < angle_cash - radians(30.0)*gear_rate){ // 30度以上回転したら
      motor.target = 0.0;
      motor.disable();
      driver.disable();
      is_moving = false;
      digitalWrite(state_Pin, LOW);
      delay(1000);
      
      send_CANdata[0] = 'L';  //キャリブレーション失敗
      send_CANdata[1] = '0';
      send_length = 2;
      while(send_flag == true);
      send_flag = true;
      while(send_flag == true);
      delay(1000);

      ESP.restart();
    }
  }
  angle_cash1 = sensor.getAngle(); // 角度を取得
  while(digitalRead(photo_Pin) == HIGH); // 黒の間ループ
  angle_cash2 = sensor.getAngle(); // 角度を取得
  while(digitalRead(photo_Pin) == LOW); // 白の間ループ
  angle_cash3 = sensor.getAngle(); // 角度を取得
  while(digitalRead(photo_Pin) == HIGH); // 黒の間ループ
  angle_cash4 = sensor.getAngle(); // 角度を取得
  while(sensor.getAngle() > angle_cash4 - radians(5.0)*gear_rate);  // 5度以上回転するまでループ
  motor.target = 0.0;
  delay(100);
  motor.target = 10.0; // 反時計回り
  while(digitalRead(photo_Pin) == LOW); // 白の間ループ
  angle_cash5 = sensor.getAngle(); // 角度を取得
  while(digitalRead(photo_Pin) == HIGH); // 黒の間ループ
  angle_cash6 = sensor.getAngle(); // 角度を取得
  while(digitalRead(photo_Pin) == LOW); // 白の間ループ
  angle_cash7 = sensor.getAngle(); // 角度を取得
  while(digitalRead(photo_Pin) == HIGH); // 黒の間ループ
  angle_cash8 = sensor.getAngle(); // 角度を取得

  motor.target = 0.0; // モーターを停止
  
  motor.disable();
  driver.disable();

  //motor.controller = MotionControlType::angle; // 角度制御モードに設定
  motor.controller = MotionControlType::torque;
  control_mode = ANGLEMODE;
  sensor_offset = (angle_cash1 + angle_cash2 + angle_cash3 + angle_cash4 + angle_cash5 + angle_cash6 + angle_cash7 + angle_cash8) / 8.0; // センサーオフセットを計算
  control_cash = 0.0;
  //motor.velocity_limit = 15.0;

  driver.enable();
  motor.enable();

  send_CANdata[0] = 'L';  //キャリブレーション成功
  send_CANdata[1] = '1';
  send_length = 2;
  while(send_flag == true);
  send_flag = true;
}