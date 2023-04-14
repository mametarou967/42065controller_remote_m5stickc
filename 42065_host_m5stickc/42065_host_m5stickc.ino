#include "PowerFunctions.h"
#include <M5StickC.h>
#include <esp_now.h>
#include <WiFi.h>

#define CMD_GOFOWARD 1
#define CMD_GOBACK 2
#define CMD_TURNRIGHT 3
#define CMD_TURNLEFT 4

/* Power Function */
PowerFunctions pf(9, 0,true /* <= Enable debug mode on pin 13? */); 
int j=0;

void goForward(uint16_t time){
  pf.combo_pwm(PWM_FWD4, PWM_REV4);
  delay(time);
  pf.combo_pwm(PWM_FLT,PWM_FLT);
}

void goBackward(uint16_t time){
  pf.combo_pwm(PWM_REV4,PWM_FWD4);
  delay(time);
  pf.combo_pwm(PWM_FLT,PWM_FLT);
}

void turnRight30Degree(uint16_t time){
  pf.combo_pwm(PWM_FWD4, PWM_FWD4);
  delay(time);
  pf.combo_pwm(PWM_FLT,PWM_FLT);  
}

void turnLeft30Degree(uint16_t time){
  pf.combo_pwm(PWM_REV4, PWM_REV4);
  delay(time);
  pf.combo_pwm(PWM_FLT,PWM_FLT);  
}

void step(uint8_t output, uint8_t pwm,  uint16_t time) {
  pf.single_pwm(output, pwm);
  delay(time);
  pf.single_pwm(output, PWM_BRK);
  delay(30);
  pf.single_pwm(output, PWM_FLT);
}


/////////////
void zigZag(uint16_t msTime){
  unsigned long startTime=millis();
  const int delayf=2;
  const int zigRange=2;
  do {
    // TO TEST THE TURNING MECHANINCS
    // Zig to left
    pf.combo_pwm(PWM_FWD4, PWM_REV2);
    delay(100*delayf*zigRange);
    goForward(50*delayf);

    // Center again
    pf.combo_pwm(PWM_FWD2, PWM_REV4);
    delay(100*delayf*zigRange);
    
    // Zig to right
    pf.combo_pwm(PWM_FWD2, PWM_REV4);
    delay(100*delayf*zigRange);   

    goForward(50* delayf);
  }while ((millis()-startTime) < msTime);
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
  goForward(100);
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
void loop() {
  M5.update();
  // ボタンを押したら送信
  if ( M5.BtnA.wasPressed() ) {
    uint8_t data[2] = {123, 234};
    esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
  }
  delay(1);
}