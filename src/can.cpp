#include "can.hpp"

volatile boolean receive_flag = false;
volatile char receive_CANdata[8]; // CAN受信データを格納する配列
volatile int receive_length = 0;
volatile int receive_id = 0; // 受信したIDを格納する変数
volatile boolean send_flag = false;
volatile char send_CANdata[8];  // CAN送信データ
volatile int send_length = 0;
volatile int send_id = 0x000; // CANのID

TaskHandle_t canTaskHandle = NULL;  // タスクハンドル

void CAN_setup(int id, uint32_t mask) {
  // CAN初期化設定
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CTX, CRX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  // 1Mbps
  //twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = {
    .acceptance_code = (mask << 21),  // 受信するID
    .acceptance_mask = ~(0b11111111111 << 21), // マスク設定(全ビット比較)
    .single_filter = true
  };

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("CAN driver install failed");
    return;
  }

  send_id = id; // CANのIDを設定

  if (twai_start() != ESP_OK) {
    Serial.println("CAN start failed");
    return;
  }

  Serial.println("CAN Started");

  // Core0にCAN受信タスクを作成
  xTaskCreatePinnedToCore(
    canTask,   // タスク関数
    "CAN",         // タスク名
    4096,             // スタックサイズ
    NULL,             // 引数
    1,                // 優先度
    &canTaskHandle,   // タスクハンドル
    0                 // Core0を指定
  );
}

void canTask(void *pvParameters) {
  twai_message_t rx_message;
  twai_message_t tx_message;

  while (true) {
    // 受信処理
    if (twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK && receive_flag == false) {
      receive_id = rx_message.identifier;
      receive_length = rx_message.data_length_code;
      for (int i = 0; i < rx_message.data_length_code; i++) {
        receive_CANdata[i] = rx_message.data[i]; // 受信データを配列に格納
      }
      receive_flag = true;
    }

    // 送信処理
    if(send_flag){
      send_flag = false; // フラグをリセット

      tx_message.identifier = send_id;
      tx_message.data_length_code = send_length;
      for (int i = 0; i < send_length; i++) {
        tx_message.data[i] = send_CANdata[i]; // シリアルから読み込んだデータをCANメッセージに設定
      }

      if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
        //Serial.println("CAN Message Sent!");
      } //else {
      //   Serial.println("CAN Send Failed");
      // }
    }

    vTaskDelay(1);
  }
}