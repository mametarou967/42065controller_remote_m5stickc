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

// TOF
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define address                                     0x29  // I2C address

byte gbuf[16];

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
  pf.combo_pwm(PWM_REV4, PWM_REV4);
}

void turnLeft(){
  pf.combo_pwm(PWM_FWD4, PWM_FWD4);
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

// ToF
uint16_t bswap(byte b[]) {
    // Big Endian unsigned short to little endian unsigned short
    uint16_t val = ((b[0] << 8) & b[1]);
    return val;
}

uint16_t makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

void write_byte_data(byte data) {
    Wire.beginTransmission(address);
    Wire.write(data);
    Wire.endTransmission();
}

void write_byte_data_at(byte reg, byte data) {
    // write data word at address and register
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void write_word_data_at(byte reg, uint16_t data) {
    // write data word at address and register
    byte b0 = (data & 0xFF);
    byte b1 = ((data >> 8) && 0xFF);

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(b0);
    Wire.write(b1);
    Wire.endTransmission();
}

byte read_byte_data() {
    Wire.requestFrom(address, 1);
    while (Wire.available() < 1) delay(1);
    byte b = Wire.read();
    return b;
}

byte read_byte_data_at(byte reg) {
    // write_byte_data((byte)0x00);
    write_byte_data(reg);
    Wire.requestFrom(address, 1);
    while (Wire.available() < 1) delay(1);
    byte b = Wire.read();
    return b;
}

uint16_t read_word_data_at(byte reg) {
    write_byte_data(reg);
    Wire.requestFrom(address, 2);
    while (Wire.available() < 2) delay(1);
    gbuf[0] = Wire.read();
    gbuf[1] = Wire.read();
    return bswap(gbuf);
}

void read_block_data_at(byte reg, int sz) {
    int i = 0;
    write_byte_data(reg);
    Wire.requestFrom(address, sz);
    for (i = 0; i < sz; i++) {
        while (Wire.available() < 1) delay(1);
        gbuf[i] = Wire.read();
    }
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
    // Converts the encoded VCSEL period register value into the real
    // period in PLL clocks
    uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
    return vcsel_period_pclks;
}

int readToF()
{
  write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

  byte val = 0;
  int cnt  = 0;
  while (cnt < 100) {  // 1 second waiting time max
      delay(10);
      val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
      if (val & 0x01) break;
      cnt++;
  }
  if (val & 0x01)
  {
    Serial.println("ready");
  }
  else
  {
    Serial.println("not ready");
    return -1;
  }

  read_block_data_at(0x14, 12);
  uint16_t acnt                  = makeuint16(gbuf[7], gbuf[6]);
  uint16_t scnt                  = makeuint16(gbuf[9], gbuf[8]);
  uint16_t dist                  = makeuint16(gbuf[11], gbuf[10]);
  byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);
  int status = (int)DeviceRangeStatusInternal;

  if(status != 11) return -1;

  return dist;  
}

int read_tof = 0;
void setup() {
  Wire.begin();  // join i2c bus (address optional for master)
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
          // mills_mode_go_stop = millis();
          goForward();
          state_mode_go_stop++;
          break;
        case STATE2_MODE_GO_STOP:
          read_tof = readToF(); 
          if((read_tof != -1) && read_tof < 175)
          {
            Serial.println("hello");
            stop();
            state_mode_go_stop++;
            Serial.println("go stop -> stop");
          }
          else
          {
            goForward();
          }
          break;
        case STATE3_MODE_GO_STOP:
          Serial.println("stop!!!");
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
          if(millis() - mills_mode_square > 500)
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
          if(millis() - mills_mode_square > 575)
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