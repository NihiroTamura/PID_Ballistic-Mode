// PWMピンの定義
#define PWM1_0 0
#define PWM1_1 1
#define PWM2_2 2
#define PWM2_3 3
#define PWM3_4 4
#define PWM3_5 5
#define PWM4_6 6
#define PWM4_7 7
#define PWM5_8 8
#define PWM5_9 9
#define PWM6_28 28
#define PWM6_29 29

//VEABのアナログ出力用ピンの定義
#define VEAB1_A0 A0
#define VEAB1_A1 A1
#define VEAB2_A2 A2
#define VEAB2_A3 A3
#define VEAB3_A4 A4
#define VEAB3_A5 A5
#define VEAB4_A6 A6
#define VEAB4_A7 A7
#define VEAB5_A8 A8
#define VEAB5_A9 A9
#define VEAB6_A10 A10
#define VEAB6_A11 A11

//POTのアナログ出力用ピンの定義
#define POT1_A12 A12
#define POT2_A13 A13
#define POT3_A14 A14
#define POT4_A15 A15
#define POT5_A16 A16
#define POT6_A17 A17

//スイッチの定義
#define Switch 33 //スタートボタン
#define Switch_emergency 34 //緊急ボタン

//POTの値を初期化
int POT1 = 0;
int POT2 = 0;
int POT3 = 0;
int POT4 = 0;
int POT5 = 0;
int POT6 = 0;

//VEABの値を初期化
int VEAB1_0 = 0;
int VEAB1_1 = 0;
int VEAB2_2 = 0;
int VEAB2_3 = 0;
int VEAB3_4 = 0;
int VEAB3_5 = 0;
int VEAB4_6 = 0;
int VEAB4_7 = 0;
int VEAB5_8 = 0;
int VEAB5_9 = 0;
int VEAB6_10 = 0;
int VEAB6_11 = 0;


void setup() {

  //シリアル通信を初期化する。ボーレートは9600bps
  Serial.begin(9600);

  //スイッチのモード設定
  pinMode(Switch, INPUT_PULLUP);
  pinMode(Switch_emergency, INPUT_PULLUP);

  //PWM周波数変更
  analogWriteFrequency(PWM1_0, 50000);
  analogWriteFrequency(PWM1_1, 50000);
  analogWriteFrequency(PWM2_2, 50000);
  analogWriteFrequency(PWM2_3, 50000);
  analogWriteFrequency(PWM3_4, 50000);
  analogWriteFrequency(PWM3_5, 50000);
  analogWriteFrequency(PWM4_6, 50000);
  analogWriteFrequency(PWM4_7, 50000);
  analogWriteFrequency(PWM5_8, 50000);
  analogWriteFrequency(PWM5_9, 50000);
  analogWriteFrequency(PWM6_28, 50000);
  analogWriteFrequency(PWM6_29, 50000);

  //VEABの初期化
  /*ピン0,1*/
  analogWrite(PWM1_0, 255);
  analogWrite(PWM1_1, 255);
  /*ピン2,3*/
  analogWrite(PWM2_2, 255);
  analogWrite(PWM2_3, 255);
  /*ピン4,5*/
  analogWrite(PWM3_4, 255);
  analogWrite(PWM3_5, 255);
  /*ピン6,7*/
  analogWrite(PWM4_6, 255);
  analogWrite(PWM4_7, 255);
  /*ピン8,9*/
  analogWrite(PWM5_8, 255);
  analogWrite(PWM5_9, 255);
  /*ピン28,29*/
  analogWrite(PWM6_28, 255);
  analogWrite(PWM6_29, 255);

  while(digitalRead(Switch) == 1){
    //何もしない処理
  }

  // 緊急ボタンの処理
  attachInterrupt(digitalPinToInterrupt(Switch_emergency), Emergency_function, FALLING);

}

void loop() {

  //POTの読み込み
  POT1 = analogRead(POT1_A12);
  POT2 = analogRead(POT2_A13);
  POT3 = analogRead(POT3_A14);
  POT4 = analogRead(POT4_A15);
  POT5 = analogRead(POT5_A16);
  POT6 = analogRead(POT6_A17);


  //PWMについて
  /*ピン0,1*/
  analogWrite(PWM1_0, 255);
  analogWrite(PWM1_1, 128);
  /*ピン2,3
  analogWrite(PWM2_2, 128);
  analogWrite(PWM2_3, 255);*/
  /*ピン4,5
  analogWrite(PWM3_4, 255);
  analogWrite(PWM3_5, 128);*/
  /*ピン6,7
  analogWrite(PWM4_6, 128);
  analogWrite(PWM4_7, 255);*/
  /*ピン8,9
  analogWrite(PWM5_8, 255);
  analogWrite(PWM5_9, 0);*/
  /*ピン28,29
  analogWrite(PWM6_28, 255);
  analogWrite(PWM6_29, 0);*/

  //VEABの読み込み
  /*ピンA0,A1
  VEAB1_0 = analogRead(VEAB1_A0);
  VEAB1_1 = analogRead(VEAB1_A1);*/
  /*ピンA2,A3
  VEAB2_2 = analogRead(VEAB2_A2);
  VEAB2_3 = analogRead(VEAB2_A3);*/
  /*ピンA4,A5
  VEAB3_4 = analogRead(VEAB3_A4);
  VEAB3_5 = analogRead(VEAB3_A5);*/
  /*ピンA6,A7
  VEAB4_6 = analogRead(VEAB4_A6);
  VEAB4_7 = analogRead(VEAB4_A7);*/
  /*ピンA8,A9
  VEAB5_8 = analogRead(VEAB5_A8);
  VEAB5_9 = analogRead(VEAB5_A9);*/
  /*ピンA10,A11
  VEAB6_10 = analogRead(VEAB6_A10);
  VEAB6_11 = analogRead(VEAB6_A11);*/

  //

  //シリアルモニタに表示
  //POT
  Serial.print("POT1:");
  Serial.print(POT1);
  Serial.print("\t POT2:");
  Serial.print(POT2);
  Serial.print("\t POT3:");
  Serial.print(POT3);
  Serial.print("\t POT4:");
  Serial.print(POT4);
  Serial.print("\t POT5:");
  Serial.print(POT5);
  Serial.print("\t POT6:");
  Serial.println(POT6);

  /*VEAB
  Serial.print("\t VEAB1_0:");
  Serial.print(VEAB1_0);
  Serial.print("\t VEAB1_1:");
  Serial.print(VEAB1_1);
  Serial.print("\t VEAB2_2:");
  Serial.print(VEAB2_2);
  Serial.print("\t VEAB2_3:");
  Serial.print(VEAB2_3);
  Serial.print("\t VEAB3_4:");
  Serial.print(VEAB3_4);
  Serial.print("\t VEAB3_5:");
  Serial.print(VEAB3_5);
  Serial.print("\t VEAB4_6:");
  Serial.print(VEAB4_6);
  Serial.print("\t VEAB4_7:");
  Serial.print(VEAB4_7);
  Serial.print("\t VEAB5_8:");
  Serial.print(VEAB5_8);
  Serial.print("\t VEAB5_9:");
  Serial.print(VEAB5_9);
  Serial.print("\t VEAB6_10:");
  Serial.print(VEAB6_10);
  Serial.print("\t VEAB6_11:");
  Serial.println(VEAB6_11);*/

}

//緊急ボタン関数
void Emergency_function(){
  //VEABの初期化
  /*ピン0,1*/
  analogWrite(PWM1_0, 255);
  analogWrite(PWM1_1, 255);
  /*ピン2,3*/
  analogWrite(PWM2_2, 255);
  analogWrite(PWM2_3, 255);
  /*ピン4,5*/
  analogWrite(PWM3_4, 255);
  analogWrite(PWM3_5, 255);
  /*ピン6,7*/
  analogWrite(PWM4_6, 255);
  analogWrite(PWM4_7, 255);
  /*ピン8,9*/
  analogWrite(PWM5_8, 255);
  analogWrite(PWM5_9, 255);
  /*ピン28,29*/
  analogWrite(PWM6_28, 255);
  analogWrite(PWM6_29, 255);

  while(1){
    //緊急時は何もしない。再度動かすときは、書き込み必要
  }
}