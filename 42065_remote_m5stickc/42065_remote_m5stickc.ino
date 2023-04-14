#include <M5StickC.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"

// JOYSTICK
#define JOY_ADDR 0x38
#define JOY_INPUT_NOP 0
#define JOY_INPUT_UP 1
#define JOY_INPUT_DOWN 2
#define JOY_INPUT_RIGHT 3
#define JOY_INPUT_LEFT 4
#define JOY_INPUT_OVER_VALUE 80
#define JOY_INPUT_UNDER_VALUE 45

void joystick_begin()
{
  Wire.begin(0,26);
}

int joystick_read()
{
  int8_t x_data = 0;
  int8_t y_data = 0;
  int8_t button_data = 1; // button data default 1
  int joy_input = JOY_INPUT_NOP;

  Wire.beginTransmission(JOY_ADDR);
  Wire.write(0x02); 
  Wire.endTransmission();
  Wire.requestFrom(JOY_ADDR, 3);
  if (Wire.available()) {
    x_data = Wire.read();
    y_data = Wire.read();
    button_data = Wire.read();
  }

  if((abs(x_data) <= JOY_INPUT_UNDER_VALUE) && (y_data >= JOY_INPUT_OVER_VALUE)) joy_input = JOY_INPUT_UP;
  else if((abs(x_data) <= JOY_INPUT_UNDER_VALUE) && (y_data <= -JOY_INPUT_OVER_VALUE)) joy_input = JOY_INPUT_DOWN;
  else if((x_data >= JOY_INPUT_OVER_VALUE) && (abs(y_data) <= JOY_INPUT_UNDER_VALUE)) joy_input = JOY_INPUT_RIGHT;
  else if((x_data <= -JOY_INPUT_OVER_VALUE) && (abs(y_data) <= JOY_INPUT_UNDER_VALUE)) joy_input = JOY_INPUT_LEFT;

  return joy_input;
}

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

  joystick_begin();
}

int joystick_input = JOY_INPUT_NOP;
int last_joystick_input = JOY_INPUT_NOP;

void loop() {
  M5.update();
  
  joystick_input = joystick_read();
  if(joystick_input != last_joystick_input)
  {
    M5.Lcd.setCursor(1, 30, 2);
    M5.Lcd.printf("joystick_input:%d\n", joystick_input);
    uint8_t data[2] = {123, joystick_input};
    esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
    last_joystick_input = joystick_input;
  }
  delay(50);
}