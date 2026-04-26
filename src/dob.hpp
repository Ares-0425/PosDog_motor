#ifndef DOB_HPP
#define DOB_HPP

// グローバル変数宣言
// PIDゲイン
extern const float velocity_P;
extern const float velocity_I;
extern const float velocity_D;
extern const float velocity_ramp;
extern const float velocity_LPF;
extern const float velocity_limit;

extern const float position_P;
extern const float position_I;
extern const float position_D;
extern const float position_ramp;
extern const float position_LPF;
extern const float position_limit;

extern const float torque_limit;

extern const float accel_LPF;

extern const float KT;
extern const int gear_rate;

struct KalmanState {
    float theta; // 推定角度 [rad]
    float omega; // 推定角速度 [rad/s]
    float alpha; // 推定角加速度 [rad/s^2]
    float P[3][3]; // 共分散
};

class KalmanEstimator {
public:
    KalmanEstimator();
    void init();
    void update(float theta_meas, float dt);
    float getTheta() const;
    float getOmega() const;
    float getAlpha() const;

private:
    KalmanState state;
    float Q[3][3];
    float R;
};

// グローバル関数宣言
// float smooth_pos(float position_now);
// float smooth_vel(float velocity_now);
// float smooth_acc(float smooth_velocity, float velocity_old, float dt);
float PID_velocity(float velocity_target, float smooth_velocity, float dt);  // 速度PID
float PID_position(float position_target, float smooth_position, float smooth_velocity, float dt);  // 位置PID
float Disturbance_OB(float torque, float smooth_accel, float smooth_velocity, float position); // 外乱オブザーバ
// float Gravity_OB();     // 重力補償器

#endif