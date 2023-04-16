#include <M5StickC.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"

// JOYSTICK
#define JOY_ADDR 0x38

#define MODE_STOP 0
#define MODE_GOFOWARD 1
#define MODE_GOBACK 2
#define MODE_TURNRIGHT 3
#define MODE_TURNLEFT 4
#define MODE_GO_STOP 5
#define MODE_SQUARE 6
#define MODE_ZIG_ZAG 7

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
  int joy_input = MODE_STOP;

  Wire.beginTransmission(JOY_ADDR);
  Wire.write(0x02); 
  Wire.endTransmission();
  Wire.requestFrom(JOY_ADDR, 3);
  if (Wire.available()) {
    x_data = Wire.read();
    y_data = Wire.read();
    button_data = Wire.read();
  }

  if((abs(x_data) <= JOY_INPUT_UNDER_VALUE) && (y_data >= JOY_INPUT_OVER_VALUE)) joy_input = MODE_GOFOWARD;
  else if((abs(x_data) <= JOY_INPUT_UNDER_VALUE) && (y_data <= -JOY_INPUT_OVER_VALUE)) joy_input = MODE_GOBACK;
  else if((x_data >= JOY_INPUT_OVER_VALUE) && (abs(y_data) <= JOY_INPUT_UNDER_VALUE)) joy_input = MODE_TURNRIGHT;
  else if((x_data <= -JOY_INPUT_OVER_VALUE) && (abs(y_data) <= JOY_INPUT_UNDER_VALUE)) joy_input = MODE_TURNLEFT;

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
  // M5.Lcd.fillScreen(BLACK);
  // M5.Lcd.setCursor(0, 0);
  // M5.Lcd.print("Last Packet Sent to: \n  ");
  // M5.Lcd.println(macStr);
  // M5.Lcd.print("Last Packet Send Status: \n  ");
  // M5.Lcd.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  // M5.Lcd.fillScreen(BLACK);
  // M5.Lcd.setCursor(0, 0);
  // M5.Lcd.print("Last Packet Recv from: \n  ");
  // M5.Lcd.println(macStr);
  // M5.Lcd.printf("Last Packet Recv Data(%d): \n  ", data_len);
  // for ( int i = 0 ; i < data_len ; i++ ) {
    // M5.Lcd.print(data[i]);
    // M5.Lcd.print(" ");
  // }
}

int menuIndex = 0;

int GetIndex()
{
  return menuIndex;
}

void UpdateIndex()
{
  menuIndex++;
  if(menuIndex > 3)
  {
    menuIndex = 0;
  }
  DispScreen();
}

void SelectIndex()
{
  int selectedCommand = MODE_STOP;
  
  if(menuIndex == 0) selectedCommand = MODE_STOP;
  else if(menuIndex == 1) selectedCommand = MODE_GO_STOP;
  else if(menuIndex == 2) selectedCommand = MODE_SQUARE;
  else if(menuIndex == 3) selectedCommand = MODE_ZIG_ZAG;
  // JoyStick情報送信
  CommandSend(selectedCommand);
  // 表示の更新
  DispScreen();
}

int joystick_input = MODE_STOP;

int GetJoyStick()
{
  return joystick_input;
}

void UpdateJoyStick(int input)
{
  // 値の更新
  joystick_input = input;
  // JoyStick情報送信
  CommandSend(joystick_input);
  // 表示の更新
  DispScreen();
}

uint8_t sentCommand = MODE_STOP;

void CommandSend(uint8_t command)
{
  uint8_t data[2] = {123, command};
  esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
  sentCommand = command;
}

int GetCommandSend()
{
  return sentCommand;
}

void DispScreen()
{
  int menuIndex = GetIndex();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("<>Menu");
  M5.Lcd.println("------------");

  if(menuIndex == 0) M5.Lcd.print(">");
  else M5.Lcd.print(" ");
  M5.Lcd.println(" STOP");
  
  if(menuIndex == 1) M5.Lcd.print(">");
  else M5.Lcd.print(" ");
  M5.Lcd.println(" GO_STOP");
  
  if(menuIndex == 2) M5.Lcd.print(">");
  else M5.Lcd.print(" ");
  M5.Lcd.println(" SQUARE");
  
  if(menuIndex == 3) M5.Lcd.print(">");
  else M5.Lcd.print(" ");
  M5.Lcd.println(" ZIG_ZAG");

  M5.Lcd.println("");

  M5.Lcd.println("<>Command");
  M5.Lcd.println("------------");

  int command = GetCommandSend();
  M5.Lcd.print("  ");
  if(command == MODE_STOP)  M5.Lcd.println("STOP");
  else if(command == MODE_GOFOWARD) M5.Lcd.println("UP");
  else if(command == MODE_GOBACK) M5.Lcd.println("DOWN");
  else if(command == MODE_TURNRIGHT) M5.Lcd.println("RIGHT");
  else if(command == MODE_TURNLEFT) M5.Lcd.println("LEFT");
  else if(command == MODE_GO_STOP) M5.Lcd.println("GO_STOP");
  else if(command == MODE_SQUARE) M5.Lcd.println("SQUARE");
  else if(command == MODE_ZIG_ZAG) M5.Lcd.println("ZIG_ZAG");
}

void setup() {
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(0);
  M5.Lcd.setTextFont(2);
  DispScreen();
  // M5.Lcd.print("ESP-NOW Test\n");
  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    // M5.Lcd.print("ESPNow Init Success\n");
  } else {
    Serial.println("ESPNow Init Failed");
    // M5.Lcd.print("ESPNow Init Failed\n");
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


void loop() {
  int read_joystick = 0;
  M5.update();
  
  // joystickの監視
  read_joystick = joystick_read();
  if(read_joystick != GetJoyStick())
  {
    UpdateJoyStick(read_joystick);
  }

  // サイドボタンの入力監視
  if ( M5.BtnB.wasPressed() )
  {
    // メニュー選択更新
    UpdateIndex();
  }

  // メインボタンの入力監視
  if ( M5.BtnA.wasPressed() )
  {
    // メニュー選択
    SelectIndex();
  }

  delay(50);
}