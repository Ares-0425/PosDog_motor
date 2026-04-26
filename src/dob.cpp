#include "dob.hpp"
#include <Arduino.h>

// PIDゲイン
const float velocity_P = 0.1;
const float velocity_I = 0.1;
const float velocity_D = 0.001;
const float velocity_ramp = 10.0;
const float velocity_LPF = 0.1;
const float velocity_limit = 0.5;

const float position_P = 9.72727567;
const float position_I = 0.0;
const float position_D = 1.01378098;
const float position_ramp = 0.0;
const float position_LPF = 0.0;
const float position_limit = 0.0;

const float torque_limit = 0.25*16.0;

const float dob_P = 0.83;

// 運動方程式関連
const float KT = 0.15;          //トルク定数(Nm/A)
const int gear_rate = 16;       //ギア比
const float mas = 0.188;        //脚重量(kg)
const float length = 0.15;      //脚質点距離(m)
const float inertia = 0.0129;   //脚慣性モーメント
const float dump = 0.0951;      //脚粘性抵抗
const float g = 9.81;           //重力加速度

// 内部状態
float smooth_position;
float smooth_velocity;
float smooth_accel;

float velocity_integral;
float velocity_error_old;

float position_integral;
float position_error_old;

//カルマンフィルタ
KalmanEstimator::KalmanEstimator() {
    // プロセスノイズの初期値（要調整）
    Q[0][0] = 1e-6; Q[0][1] = 0;     Q[0][2] = 0;
    Q[1][0] = 0;    Q[1][1] = 1e-4;  Q[1][2] = 0;
    Q[2][0] = 0;    Q[2][1] = 0;     Q[2][2] = 1e-2;

    // 観測ノイズ（エンコーダの分解能によって調整）
    R = 1e-4;
}

void KalmanEstimator::init() {
    state.theta = 0;
    state.omega = 0;
    state.alpha = 0;
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            state.P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void KalmanEstimator::update(float theta_meas, float dt) {
    // --- 予測 ---
    float theta_pred = state.theta + dt*state.omega + 0.5f*dt*dt*state.alpha;
    float omega_pred = state.omega + dt*state.alpha;
    float alpha_pred = state.alpha;

    float F[3][3] = {
        {1, dt, 0.5f*dt*dt},
        {0, 1, dt},
        {0, 0, 1}
    };

    // P_pred = F * P * F^T + Q
    float P_pred[3][3] = {0};
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            for (int k=0; k<3; k++) {
                P_pred[i][j] += F[i][k]*state.P[k][j];
            }
        }
    }

    float P_temp[3][3] = {0};
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            for (int k=0; k<3; k++) {
                P_temp[i][j] += P_pred[i][k]*F[j][k];
            }
            P_temp[i][j] += Q[i][j];
        }
    }

    // --- 更新 ---
    float y = theta_meas - theta_pred; // 残差
    float S = P_temp[0][0] + R;
    float K[3] = {
        P_temp[0][0]/S,
        P_temp[1][0]/S,
        P_temp[2][0]/S
    };

    state.theta = theta_pred + K[0]*y;
    state.omega = omega_pred + K[1]*y;
    state.alpha = alpha_pred + K[2]*y;

    // 共分散更新（簡略版）
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            state.P[i][j] = P_temp[i][j] - K[i]*P_temp[0][j];
        }
    }
}

float KalmanEstimator::getTheta() const { return state.theta; }
float KalmanEstimator::getOmega() const { return state.omega; }
float KalmanEstimator::getAlpha() const { return state.alpha; }

/*
float smooth_pos(float position_now){
    smooth_position = position_LPF * smooth_position + (1.0 - position_LPF) * position_now;

    return smooth_position;
}
float smooth_vel(float velocity_now){
    smooth_velocity = velocity_LPF * smooth_velocity + (1.0 - velocity_LPF) * velocity_now;

    return smooth_velocity;
}
float smooth_acc(float smooth_velocity, float velocity_old, float dt){
    float accel_now = (smooth_velocity - velocity_old) / dt;
    smooth_accel = accel_LPF * smooth_accel + (1.0 - accel_LPF) * accel_now;

    return smooth_accel;
}
*/

float PID_velocity(float velocity_target, float smooth_velocity, float dt){
    // 速度ランプ制限
    float delta = velocity_target - smooth_velocity;
    float max_delta = velocity_ramp * dt;
    if(delta > max_delta) delta = max_delta;
    if(delta < -max_delta) delta = -max_delta;
    float target_filtered = smooth_velocity + delta;

    // 誤差計算
    float error = target_filtered - smooth_velocity;

    // 積分項
    velocity_integral += error * dt;

    // 微分項
    float d_raw = (error - velocity_error_old) / dt;
    velocity_error_old = error;

    // PID制御
    float torque = velocity_P * error + velocity_I * velocity_integral + velocity_D * d_raw;

    // 出力制限
    if(torque > torque_limit) torque = torque_limit;
    if(torque < -torque_limit) torque = -torque_limit;

    return torque;
}

float PID_position(float position_target, float smooth_position, float smooth_velocity, float dt){
    // 位置ランプ制限
    // float delta = position_target - smooth_position;
    // float max_delta = position_ramp * dt;
    // if(delta > max_delta) delta = max_delta;
    // if(delta < -max_delta) delta = -max_delta;
    // float target_filtered = smooth_position + delta;

    // 誤差計算
    float error = position_target - smooth_position;

    // 積分項
    // position_integral += error * dt;

    // 微分項
    // float d_raw = (error - position_error_old) / dt;
    // position_error_old = error;

    // PID制御
    float torque = (position_P * error + position_D*0 * smooth_velocity);

    //Serial.printf("P:%f %f %f\n",torque,error,smooth_velocity);

    // 出力制限
    if(torque > torque_limit) torque = torque_limit;
    if(torque < -torque_limit) torque = -torque_limit;

    return torque;
}

float Disturbance_OB(float torque, float smooth_accel, float smooth_velocity, float position){
    // トルク計算値
    float torque_est = inertia * smooth_accel + dump * smooth_velocity + mas*g*length*sin(position);
     // トルク推定値　=　慣性モーメント*角加速度　+　粘性抵抗*角速度　+　重量*重力加速度*質点距離*角度

    //外乱トルク
    float torque_ext = torque - torque_est; // 外乱トルク　=　入力トルク　-　トルク計算値

    return dob_P*torque_ext;
}