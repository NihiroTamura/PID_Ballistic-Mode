// 青色LEDのピン接続番号
#define LED_BLUE  30

// スイッチのピン接続番号
#define SWITCH_HIDARI  16
#define SWITCH_CHUO    18
#define SWITCH_MIGI    22

// スイッチの状態
#define OFF 1
#define ON  0

// 点灯時間(単位:ms)
#define TENTOU_JIKAN 50


void setup() {
  // LEDのピンを出力に設定
  pinMode(LED_BLUE, OUTPUT);

  // スイッチのピンを入力に設定
  pinMode(SWITCH_HIDARI, INPUT_PULLUP);

  // 最初はLEDをOFFに設定しておく
  digitalWrite(LED_BLUE, LOW);

  // シリアルモニタ設定
  Serial.begin(9600);

  // シリアルモニタ待ち
  while(!Serial){
  }
}


void loop() {

  //前回と今回のスイッチ状態を記録する変数
  uint8_t konkai;
  static uint8_t zenkai = OFF;

  konkai = digitalRead(SWITCH_HIDARI);

  // 左スイッチ状態を読み取り、ONだったらLEDを一定時間点灯する
  if( (zenkai == ON) && (konkai == OFF) ) {
    // LEDを点灯
    digitalWrite(LED_BLUE, HIGH);
    // 一定時間待つ
    delay(TENTOU_JIKAN);
    // LEDを消す
    digitalWrite(LED_BLUE, LOW);
  }

  zenkai = konkai;
  
}