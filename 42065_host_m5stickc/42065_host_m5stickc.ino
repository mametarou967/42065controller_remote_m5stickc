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
#define CMD_GO_STOP 5
#define CMD_SQUARE 6
#define CMD_ZIG_ZAG 7

#define MODE_STOP 0
#define MODE_GOFOWARD 1
#define MODE_GOBACK 2
#define MODE_TURNRIGHT 3
#define MODE_TURNLEFT 4
#define MODE_GO_STOP 5
#define MODE_SQUARE 6
#define MODE_ZIG_ZAG 7

#define STATE1_MODE_GO_STOP 1
#define STATE2_MODE_GO_STOP 2
#define STATE3_MODE_GO_STOP 3

#define STATE1_MODE_SQUARE 1
#define STATE2_MODE_SQUARE 2
#define STATE3_MODE_SQUARE 3
#define STATE4_MODE_SQUARE 4

int state_mode_go_stop = STATE1_MODE_GO_STOP;
long mills_mode_go_stop = 0;
int state_mode_square = STATE1_MODE_SQUARE;
long mills_mode_square = 0;
int count_mode_squre = 0;

ArduinoQueue<int> queue(16); // 適当なサイズ

/* Power Function */
PowerFunctions pf(9, 0,true /* <= Enable debug mode on pin 13? */); 
int j=0;

void stop()
{
  Serial.println("stop");
  pf.combo_pwm(PWM_FLT,PWM_FLT); 
}

void goForward(){
  Serial.println("goForward");
  pf.combo_pwm(PWM_FWD4, PWM_REV4);
}

void goBackward(){
  Serial.println("goBackward");
  pf.combo_pwm(PWM_REV4,PWM_FWD4);
}

void turnRight(){
  Serial.println("turnRight");
  pf.combo_pwm(PWM_FWD4, PWM_FWD4);
}

void turnLeft(){
  Serial.println("turnLeft");
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
  for ( int i = 0 ; i < data_len ; i++ ) {
    // M5.Lcd.print(data[i]);
    // M5.Lcd.print(" ");
  }
  queue.enqueue(data[1]);
}

int mode = MODE_STOP;

void DispScreen()
{
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("<>Mode");
  M5.Lcd.println("--------------------------");
  
  M5.Lcd.print("  ");
  if(mode == MODE_STOP)  M5.Lcd.println("STOP");
  else if(mode == MODE_GOFOWARD) M5.Lcd.println("UP");
  else if(mode == MODE_GOBACK) M5.Lcd.println("DOWN");
  else if(mode == MODE_TURNRIGHT) M5.Lcd.println("RIGHT");
  else if(mode == MODE_TURNLEFT) M5.Lcd.println("LEFT");
  else if(mode == MODE_GO_STOP) M5.Lcd.println("GO_STOP");
  else if(mode == MODE_SQUARE) M5.Lcd.println("SQUARE");
  else if(mode == MODE_ZIG_ZAG) M5.Lcd.println("ZIG_ZAG");
}

void modeUpdate(int receiveCommand)
{
    switch(receiveCommand)
    {
      case CMD_NOP: mode = MODE_STOP; break;
      case CMD_GOFOWARD:  mode = MODE_GOFOWARD; break;
      case CMD_GOBACK:  mode = MODE_GOBACK; break;
      case CMD_TURNRIGHT:  mode = MODE_TURNRIGHT; break;
      case CMD_TURNLEFT:  mode = MODE_TURNLEFT; break;
      case CMD_GO_STOP:  
        mode = MODE_GO_STOP;
        state_mode_go_stop = STATE1_MODE_GO_STOP;
        break;
      case CMD_SQUARE:  
        mode = MODE_SQUARE;
        state_mode_square = STATE1_MODE_SQUARE;
        break;
      case CMD_ZIG_ZAG:  mode = MODE_ZIG_ZAG; break;
      default: mode = MODE_STOP; break;
    }
    DispScreen();
}

void setup() {
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextFont(2);
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

  DispScreen();
}




void loop() {
  int receive_cmd = 0;
  bool modeUpdateFlg;

  modeUpdateFlg = false;

  M5.update();
  // コマンドを受信したら
  if(!queue.isEmpty()){
    receive_cmd = queue.dequeue();
    Serial.printf("receive_cmd = %d\n",receive_cmd);
    Serial.printf("mode = %d\n",mode);
    modeUpdate(receive_cmd);
    modeUpdateFlg = true;
  }

  switch(mode)
  {
    case MODE_STOP:  if(modeUpdateFlg){stop();} break;
    case MODE_GOFOWARD:  goForward(); break;
    case MODE_GOBACK:  goBackward(); break;
    case MODE_TURNRIGHT:  turnRight(); break;
    case MODE_TURNLEFT:  turnLeft(); break;
    case MODE_GO_STOP:
      switch(state_mode_go_stop)
      {
        case STATE1_MODE_GO_STOP:
          mills_mode_go_stop = millis();
          goForward();
          state_mode_go_stop++;
          break;
        case STATE2_MODE_GO_STOP:
          if(millis() - mills_mode_go_stop > 2000)
          {
            stop();
            state_mode_go_stop++;
          }
          else
          {
            goForward();
          }
          break;
        case STATE3_MODE_GO_STOP:
          queue.enqueue(CMD_NOP);
          break;
      }
      break;
    case MODE_SQUARE:
      switch(state_mode_square)
      {
        case STATE1_MODE_SQUARE:
          count_mode_squre = 0;
          state_mode_square++;
          break;
        case STATE2_MODE_SQUARE:
          mills_mode_square = millis();
          goForward();
          state_mode_square++;
          break;
        case STATE3_MODE_SQUARE:
          if(millis() - mills_mode_square > 2000)
          {
            mills_mode_square = millis();
            turnRight();
            state_mode_square++;
          }
          else
          {
            goForward();
          }
          break;
        case STATE4_MODE_SQUARE:
          if(millis() - mills_mode_square > 2000)
          {
            // 1セット終了
            count_mode_squre++;
            if(count_mode_squre > 3)
            {
              // 終了
              queue.enqueue(CMD_NOP);
            } 
            else
            {           
              mills_mode_square = millis();
              goForward();
              state_mode_square = STATE3_MODE_SQUARE;
            }
          }
          else
          {
            turnRight();
          }
          break;
      }
      break;
    default:  if(modeUpdateFlg){stop();} break;
  }

  delay(1);
}