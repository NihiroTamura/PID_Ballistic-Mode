/*キッチンタイマー33まで*/
//ピン番号の定義
#include "teigi.h"

void setup(){
  pinMode(PWM_RED , OUTPUT);
  pinMode(PWM_GREEN, OUTPUT);
  pinMode(PWM_YELLOW, OUTPUT);
  pinMode(PWM_BLUE, OUTPUT);
  pinMode(Switch, INPUT_PULLUP);
  pinMode(Speaker, OUTPUT);
  Serial.begin(9600);

  //スイッチが押されるまで待つ
  while(digitalRead(Switch) == 1 ){
  }
}

void loop()
{
  uint8_t count;

  //countの初期化
  count = 0;

  //LED
  for(count = 0; count < Timer_count+1; count+=1){

    if(count == 10){
      digitalWrite(PWM_RED, HIGH);

      //ドの音
      for(uint16_t count_speaker = 0; count_speaker < 786; count_speaker++){
        digitalWrite(Speaker, HIGH);
        delayMicroseconds(1908);
        digitalWrite(Speaker, LOW);
        delayMicroseconds(1909);
      }
    }

    if(count == 20){
      digitalWrite(PWM_YELLOW, HIGH);
      digitalWrite(PWM_RED, LOW);

      //ソの音
      for(uint16_t count_speaker = 0; count_speaker < 786; count_speaker++){
        digitalWrite(Speaker, HIGH);
        delayMicroseconds(1275);
        digitalWrite(Speaker, LOW);
        delayMicroseconds(1276);
      }
    }

    if(count > 27 && count < 30){
      if(count == 28){
        tone(Speaker, Alarm);
        noTone(Speaker);
      }

      if(count == 29){
        tone(Speaker, Alarm);
        delay(60);
        noTone(Speaker);
        delay(60);
      }
    }

    if(count == 30){
      digitalWrite(PWM_YELLOW, LOW);
      digitalWrite(PWM_GREEN, HIGH);

      //1オクターブ高いド
      for(uint16_t count_speaker = 0; count_speaker < 786; count_speaker++){
        digitalWrite(Speaker, HIGH);
        delayMicroseconds(956);
        digitalWrite(Speaker, LOW);
        delayMicroseconds(956);
      }
    }

    
    digitalWrite(PWM_BLUE, HIGH);
    delay(Timer_jikan_on);
    digitalWrite(PWM_BLUE, LOW);
    delay(Timer_jikan_off);
    Serial.println(digitalRead(Switch));
  }

  while(true){
    digitalWrite(PWM_GREEN, LOW);
    Serial.println("終了");
    delay(Read_Jikan);
  }
}