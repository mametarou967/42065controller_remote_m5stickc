#include "PowerFunctions.h"
#include <M5StickC.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoQueue.h>

#define CMD_NOP 0
#define CMD_GOFOWARD 1
#define CMD_GOBACK 2
#define CMD_TURNRIGHT 3
#define CMD_TURNLEFT 4

#define MODE_STOP 0
#define MODE_GOFOWARD 1
#define MODE_GOBACK 2
#define MODE_TURNRIGHT 3
#define MODE_TURNLEFT 4

ArduinoQueue<int> queue(16); // 適当なサイズ

/* Power Function */
PowerFunctions pf(9, 0,true /* <= Enable debug mode on pin 13? */); 
int j=0;

void stop()
{
  pf.combo_pwm(PWM_FLT,PWM_FLT); 
}

void goForward(){
  pf.combo_pwm(PWM_FWD4, PWM_REV4);
}

void goBackward(){
  pf.combo_pwm(PWM_REV4,PWM_FWD4);
}

void turnRight(){
  pf.combo_pwm(PWM_FWD4, PWM_FWD4);
}

void turnLeft(){
  pf.combo_pwm(PWM_REV4, PWM_REV4);
}

// esp-now
esp_now_peer_info_t slave;
// 送信コールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // 画面にも描画
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("Last Packet Sent to: \n  ");
  M5.Lcd.println(macStr);
  M5.Lcd.print("Last Packet Send Status: \n  ");
  M5.Lcd.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("Last Packet Recv from: %s\n", macStr);
  Serial.printf("Last Packet Recv Data(%d): ", data_len);
  for ( int i = 0 ; i < data_len ; i++ ) {
    Serial.print(data[i]);
    Serial.print(" ");
  }
  Serial.println("");
  // 画面にも描画
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.print("Last Packet Recv from: \n  ");
  M5.Lcd.println(macStr);
  M5.Lcd.printf("Last Packet Recv Data(%d): \n  ", data_len);
  for ( int i = 0 ; i < data_len ; i++ ) {
    M5.Lcd.print(data[i]);
    M5.Lcd.print(" ");
  }
  queue.enqueue(data[1]);
}
void setup() {
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);
  M5.Lcd.print("ESP-NOW Test\n");
  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    M5.Lcd.print("ESPNow Init Success\n");
  } else {
    Serial.println("ESPNow Init Failed");
    M5.Lcd.print("ESPNow Init Failed\n");
    ESP.restart();
  }
  // マルチキャスト用Slave登録
  memset(&slave, 0, sizeof(slave));
  for (int i = 0; i < 6; ++i) {
    slave.peer_addr[i] = (uint8_t)0xff;
  }
  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }
  // ESP-NOWコールバック登録
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

int mode = MODE_STOP;
int last_mode = MODE_STOP;
void loop() {
  int receive_cmd = 0;

  M5.update();
  // ボタンを押したら送信
  if(!queue.isEmpty()){
    receive_cmd = queue.dequeue();
    Serial.printf("receive_cmd = %d\n",receive_cmd);
    switch(receive_cmd)
    {
      case CMD_NOP: mode = MODE_STOP; break;
      case CMD_GOFOWARD:  mode = MODE_GOFOWARD; break;
      case CMD_GOBACK:  mode = MODE_GOBACK; break;
      case CMD_TURNRIGHT:  mode = MODE_TURNRIGHT; break;
      case CMD_TURNLEFT:  mode = MODE_TURNLEFT; break;
      default: mode = MODE_STOP; break;
    }
    Serial.printf("mode = %d last_mode = %d\n",mode,last_mode);
  }

  if(last_mode != mode)
  {
    switch(mode)
    {
      case MODE_STOP:  stop(); break;
      case MODE_GOFOWARD:  goForward(); break;
      case MODE_GOBACK:  goBackward(); break;
      case MODE_TURNRIGHT:  turnRight(); break;
      case MODE_TURNLEFT:  turnLeft(); break;
      default:  stop(); break;
    }
    // 前回モードの更新
    last_mode = mode;
  }
  delay(1);
}