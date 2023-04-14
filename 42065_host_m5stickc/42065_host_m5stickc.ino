
// -*- mode:c++; mode: flymake -*-
#include "PowerFunctions.h"
#include <M5StickC.h>



/*** 
 *  Zig Zag a RC Tracked Racer!
 *  Version 1.1
 *  
 * This demo uses an arduino to drive a 
 * LEGO RC Tracked Racer 42065
 * To drive it you must give inverted command in sync.
 * 
 * Wiring: wire 4 buttons with different resistence to pin A0, using the keyboard example 7 in ardruino starter kit
 * Connect the led to pin 12.
 * 
 * Alternate Wiring: Use a mosfet to drive more power to the led (use pin 12 to control the mosfet.
 * 
 * 
 */

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

boolean stop=true;

void setup() {
  M5.begin();
  M5.Lcd.setRotation(3); // 画面に入らないので横向きにする
  // put your setup code here, to run once:  
  M5.Lcd.println("Hello");
}

int mode = 0;
void loop() {
  // Buttonクラスを利用するときには必ずUpdateを呼んで状態を更新する
  M5.update();

  // カーソル初期化
  M5.Lcd.setCursor(0, 0);
  
  if(M5.BtnA.isPressed())
  {
    if(mode == 0) goForward(100);
    else if(mode == 1) goBackward(100);
    else if(mode == 2) turnRight30Degree(100);
    else if(mode == 3) turnLeft30Degree(100);
  }

  if(M5.BtnB.isPressed())
  {
    mode++;
    if(mode == 4) mode = 0;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("mode=%d",mode);
  }  

  delay(100);
}