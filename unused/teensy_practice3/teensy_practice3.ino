// 音階定義ファイルのインクルード
#include "onkai.h"

// 秒を表現するLED関連(青色LED)
#define BYOU_LED 12 // 秒を表現する青色LEDの端子番号
#define BYOU_ON  50 // 秒を表現するLEDをつけている時間 (単位:ミリ秒)
#define BYOU_OFF 1000 - BYOU_ON // 秒を表現するLEDを消している時間 (単位:ミリ秒)

// 残り時間を表現するLED関連(緑色、黄色、赤色LED)
#define LED_MIDORI 8 // 緑色LEDの端子番号
#define LED_KIIRO  6 // 黄色LEDの端子番号
#define LED_AKA    4 // 赤色LEDの端子番号

// スタートスイッチ関連
#define SWITCH 23    // スイッチを接続している端子番号
#define SWITCH_OFF 1 // スイッチOFFの時のdigitalReadの値
#define SWITCH_ON  0 // スイッチONの時のdigitalReadの値

// タイマー時間設定(単位:秒)
#define TIMER_JIKAN 10

// 残り時間を表現するLEDの制御時間
#define KIIRO_JIKAN TIMER_JIKAN / 3
#define AKA_JIKAN   TIMER_JIKAN / 3 * 2

// アラーム音関連
#define SPEAKER 18  // スピーカーの端子番号
#define ALARM   880 // 通常のアラーム音周波数

#define ALARM_ONKAISUU 13 // アラームメロディーの音の数
uint16_t alarm_melody[] = {DO_4, MI_4, SO_4, MI_4, DO_4, MI_4, SO_4, MI_4, DO_4, MI_4, SO_4, SI_4, DO_5};  // アラームメロディー


// 残り時間表示用LEDの制御関数
//   パラメータ: HIGHまたはLOW
//       midori: 緑色LEDのON/OFF
//       kiiro:  黄色LEDのON/OFF
//       aka:    赤色LEDのON/OFF
//   返り値:
//       なし
//
void controlTimeLed(uint8_t midori, uint8_t kiiro, uint8_t aka) {
  digitalWrite(LED_MIDORI, midori);
  digitalWrite(LED_KIIRO,  kiiro);
  digitalWrite(LED_AKA,    aka);
}


void setup() {
  // 端子の設定
  pinMode(BYOU_LED,   OUTPUT);   // 青色LED接続端子設定
  pinMode(LED_MIDORI, OUTPUT);   // 緑色LED接続端子設定
  pinMode(LED_KIIRO,  OUTPUT);   // 緑色LED接続端子設定
  pinMode(LED_AKA,    OUTPUT);   // 緑色LED接続端子設定
  pinMode(SWITCH, INPUT_PULLUP); // スイッチ接続端子の設定

  // 秒のLED(青色LED)を点灯する
  digitalWrite(BYOU_LED, HIGH);

  // スイッチが押されるまで待つ
  while(digitalRead(SWITCH) == SWITCH_OFF) {
  }

  // タイマー開始時に緑色LEDを点灯
  controlTimeLed(HIGH, LOW, LOW);

}


void loop() {
  
  uint8_t count;       // for文で回数を数えるために使用する変数
  uint8_t oto_bangou;  // for文でメロディーを鳴らすために使用する変数
  

  // TIMER_JIKAN分の回数を数える
  for( count=0; count<TIMER_JIKAN; count++) {

    // 残り時間表現用LEDの制御
    if( count == KIIRO_JIKAN ) {
      controlTimeLed(LOW, HIGH, LOW);
    }

    if( count == AKA_JIKAN ){
      controlTimeLed(LOW, LOW, HIGH);
    }

    // 残り時間に応じて処理を変える
    if( count >= (TIMER_JIKAN - 5) ) {
      // 残り時間が5秒以下になったら、1秒に1回青色LEDを点滅して音を鳴らす
      digitalWrite(BYOU_LED, HIGH);
      tone(SPEAKER, ALARM);
      delay(BYOU_ON);
      digitalWrite(BYOU_LED, LOW);
      noTone(SPEAKER);
      delay(BYOU_OFF);
    } else {
      // そうでなければ、1秒に1回青色LEDを点滅する
      digitalWrite(BYOU_LED, HIGH);
      delay(BYOU_ON);
      digitalWrite(BYOU_LED, LOW);
      delay(BYOU_OFF);
    }
    
  }

  // 時間になったので秒のLED(青色LED)を点灯する
  digitalWrite(BYOU_LED, HIGH);

  // メロディー音を3回鳴らす
  for( count=0; count<3; count++) {

    // メロディーを演奏する
    for( oto_bangou=0; oto_bangou<ALARM_ONKAISUU; oto_bangou++ ) {
      tone(SPEAKER, alarm_melody[oto_bangou]);
      delay(200);
    }
    
    // 音を消す
    noTone(SPEAKER);

    // 1.5秒あける
    delay(1500);

  }

  // 何もしないで待つ
  while( true ) {
  }

}