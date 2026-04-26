#ifndef CAN_HPP
#define CAN_HPP

#include <Arduino.h>
#include <driver/twai.h>

#define CRX GPIO_NUM_4  // CAN受信ピン
#define CTX GPIO_NUM_16 // CAN送信ピン

// グローバル変数宣言
extern volatile boolean receive_flag;
extern volatile char receive_CANdata[8]; // CAN受信データを格納する配列
extern volatile int receive_length;
extern volatile int receive_id; // 受信したIDを格納する変数
extern volatile boolean send_flag;
extern volatile char send_CANdata[8];  // CAN送信データ
extern volatile int send_length;
extern volatile int send_id; // CANのID

// グローバル関数宣言
void CAN_setup(int, uint32_t);
void canTask(void *pvParameters);

#endif