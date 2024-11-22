/*
2024/11/19
PID制御とBallistic Modeの実装
ROS2はなし

2024/11/21
ローパスフィルタ適用(POT, VEAB)

--注意--
使うTeensyによって、
1.PIDゲイン
2.Ballistic Modeのパラメータ値
3.圧力の正方向パラメータdirection
を変更！
*/

/*------ピンの定義----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
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

/*------変数の初期化----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//POTの値を初期化
int POT1 = 0;
int POT2 = 0;
int POT3 = 0;
int POT4 = 0;
int POT5 = 0;
int POT6 = 0;

int POT_realized[6] = {
  0, 0, 0, 0, 0, 0
};

/*VEABの値を初期化
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
int VEAB6_11 = 0;*/

/*---PID制御用-----------------------------------------------------------------------------------------*/
//PIDゲイン
const float kp[6] = {
  1.0, 3.0, 0.8, 0.35, 0.0, 0.0
};
const float ki[6] = {
  0.0, 0.0, 0.01, 0.002, 0.0, 0.0
};
const float kd[6] = {
  10.0, 12.0, 10.0, 4.0, 0.0, 0.0
};

//各自由度ごとの圧力の正方向とポテンショメータの正方向の対応を整理
const int direction[6] = {
  -1, -1, 1, -1, -1, -1
};

//各要素(自由度)に対する1.誤差、2.前回の誤差、3.誤差の積分値、4.誤差の微分値
int errors[6] = {
  0, 0, 0, 0, 0, 0
};

int previous_errors[6] = {
  0, 0, 0, 0, 0, 0
};

int integral[6] = {
  0, 0, 0, 0, 0, 0
};

int de[6] = {
  0, 0, 0, 0, 0, 0
};

//目標値の設定
int POT_desired[6] = {
  320, 365, 113, 260, 950, 470
};//{腕の閉223-482開, 腕の下344-619上, 上腕の旋回内95-605外, 肘の伸144-740曲, 前腕の旋回内111-962外, 小指側縮62-895伸}

//PID出力値
float outputPID[6] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

/*---VEABへのPWM信号の出力-----------------------------------------------------------------------------------------*/
//VEABへのPWM出力値計算用構造体
struct Result {
  int veab_value1;
  int veab_value2;
};

//VEABへのPWM出力値
int VEAB_desired[12] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*------ローパスフィルタ----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//ローパスフィルタの係数
const float coef_lpf_veab = 0.7; //VEAB
const float coef_lpf_pot = 0.8;   //POT

//ローパスフィルタの値保持変数
int veab_filter[12] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
}; //VEAB
int pot_filter[6] = {
  0, 0, 0, 0, 0, 0
}; //POT

//ローパスフィルタ用前回の値保持変数
int previous_value_veab[12] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
}; //VEAB
int previous_value_pot[6] = {
  0, 0, 0, 0, 0, 0
}; //POT

//ローパスフィルタ用初回判定
int initial_lpf_veab[12] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
}; //VEAB
int initial_lpf_pot[6] = {
  0, 0, 0, 0, 0, 0
}; //POT


/*------プログラムスタート----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
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

  //POT値をシリアルモニタに表示(初回)および誤差(前回分)の初期化
  while(digitalRead(Switch) == 1){
    //POT値をPOT_realizedに格納
    POT_realized[0] = analogRead(POT1_A12);
    POT_realized[1] = analogRead(POT2_A13);
    POT_realized[2] = analogRead(POT3_A14);
    POT_realized[3] = analogRead(POT4_A15);
    POT_realized[4] = analogRead(POT5_A16);
    POT_realized[5] = analogRead(POT6_A17);

    //ローパスフィルタPOT(移動平均)
    for(int kaisuu = 0; kaisuu < 99; kaisuu++){
      POT_realized[0] += analogRead(POT1_A12);
      POT_realized[1] += analogRead(POT2_A13);
      POT_realized[2] += analogRead(POT3_A14);
      POT_realized[3] += analogRead(POT4_A15);
      POT_realized[4] += analogRead(POT5_A16);
      POT_realized[5] += analogRead(POT6_A17);
    }

    POT_realized[0] = POT_realized[0] /100;
    POT_realized[1] = POT_realized[1] /100;
    POT_realized[2] = POT_realized[2] /100;
    POT_realized[3] = POT_realized[3] /100;
    POT_realized[4] = POT_realized[4] /100;
    POT_realized[5] = POT_realized[5] /100;

    //シリアルモニタに表示
    //POT
    Serial.print("POT1:");
    Serial.print(POT_realized[0]);
    Serial.print("\t POT2:");
    Serial.print(POT_realized[1]);
    Serial.print("\t POT3:");
    Serial.print(POT_realized[2]);
    Serial.print("\t POT4:");
    Serial.print(POT_realized[3]);
    Serial.print("\t POT5:");
    Serial.print(POT_realized[4]);
    Serial.print("\t POT6:");
    Serial.println(POT_realized[5]);

    //誤差(前回分)の初期化
    for(int i = 0; i < 6; i++){
      previous_errors[i] = POT_desired[i] - POT_realized[i];
    }

  }

  // 緊急ボタンの処理
  attachInterrupt(digitalPinToInterrupt(Switch_emergency), Emergency_function, FALLING);

}


/*------ループ処理----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() {

  /*------PID制御-----------------------------------------------------------------------------------------*/
  //POT値をPOT_realizedに格納
  POT_realized[0] = analogRead(POT1_A12);
  POT_realized[1] = analogRead(POT2_A13);
  POT_realized[2] = analogRead(POT3_A14);
  POT_realized[3] = analogRead(POT4_A15);
  POT_realized[4] = analogRead(POT5_A16);
  POT_realized[5] = analogRead(POT6_A17);

  //ローパスフィルタ適用(POT)
  /*for(int i = 0; i < 6; i++){
    
    //ローパスフィルタ関数呼び出し
    pot_filter[i] = lpf_function(POT_realized[i], previous_value_pot[i], initial_lpf_pot[i], coef_lpf_pot);

    initial_lpf_pot[i] += 1;

    //POT値に格納
    POT_realized[i] = pot_filter[i];

    //前回のPOT値に格納
    previous_value_pot[i] = pot_filter[i];
    
  }*/

  //ローパスフィルタPOT(移動平均)
  for(int kaisuu = 0; kaisuu < 99; kaisuu++){
    POT_realized[0] += analogRead(POT1_A12);
    POT_realized[1] += analogRead(POT2_A13);
    POT_realized[2] += analogRead(POT3_A14);
    POT_realized[3] += analogRead(POT4_A15);
    POT_realized[4] += analogRead(POT5_A16);
    POT_realized[5] += analogRead(POT6_A17);
  }

  POT_realized[0] = POT_realized[0] /100;
  POT_realized[1] = POT_realized[1] /100;
  POT_realized[2] = POT_realized[2] /100;
  POT_realized[3] = POT_realized[3] /100;
  POT_realized[4] = POT_realized[4] /100;
  POT_realized[5] = POT_realized[5] /100;


  //PID制御
  for(int i = 0; i < 6; i++){

    //誤差計算
    errors[i] = POT_desired[i] - POT_realized[i];

    //誤差の積分値計算
    integral[i] += errors[i];

    //誤差の微分値計算
    de[i] = errors[i] - previous_errors[i];

    //PID制御計算
    outputPID[i] = (kp[i] * errors[i] + ki[i] * integral[i] + kd[i] * de[i]) * direction[i];

    //VEAB1とVEAB2に与えるPWMの値を計算し格納
    Result veab = calculate_veab_Values(outputPID[i], i);
    VEAB_desired[2*i] = veab.veab_value1;   //0, 2, 4, 6, 8, 10ピンへ
    VEAB_desired[2*i+1] = veab.veab_value2; //1, 3, 5, 7, 9, 11ピンへ

    //計算に用いた誤差を前回の誤差に変更
    previous_errors[i] = errors[i];

  }

  //ローパスフィルタ適用(VEAB)
  for(int i = 0; i < 12; i++){
    
    //ローパスフィルタ関数呼び出し
    veab_filter[i] = lpf_function(VEAB_desired[i], previous_value_veab[i], initial_lpf_veab[i], coef_lpf_veab);

    initial_lpf_veab[i] += 1;

    //PWM値に格納
    VEAB_desired[i] = veab_filter[i];

    //前回のVEAB値に格納
    previous_value_veab[i] = veab_filter[i];
    
  }

  /*------VEABへ出力-----------------------------------------------------------------------------------------*/
  /*ピン0,1
  analogWrite(PWM1_0, VEAB_desired[0]);
  analogWrite(PWM1_1, VEAB_desired[1]);*/
  /*ピン2,3
  analogWrite(PWM2_2, VEAB_desired[2]);
  analogWrite(PWM2_3, VEAB_desired[3]);*/
  /*ピン4,5
  analogWrite(PWM3_4, VEAB_desired[4]);
  analogWrite(PWM3_5, VEAB_desired[5]);*/
  /*ピン6,7*/
  analogWrite(PWM4_6, VEAB_desired[6]);
  analogWrite(PWM4_7, VEAB_desired[7]);
  /*ピン8,9
  analogWrite(PWM5_8, VEAB_desired[8]);
  analogWrite(PWM5_9, VEAB_desired[9]);*/
  /*ピン28,29
  analogWrite(PWM6_28, VEAB_desired[10]);
  analogWrite(PWM6_29, VEAB_desired[11]);*/

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


  /*------シリアルモニタに表示-----------------------------------------------------------------------------------------*/
  /*POT*/
  Serial.print("POT1:");
  Serial.print(POT_realized[0]);
  Serial.print("\t POT2:");
  Serial.print(POT_realized[1]);
  Serial.print("\t POT3:");
  Serial.print(POT_realized[2]);
  Serial.print("\t POT4:");
  Serial.print(POT_realized[3]);
  Serial.print("\t POT5:");
  Serial.print(POT_realized[4]);
  Serial.print("\t POT6:");
  Serial.print(POT_realized[5]);

  /*VEAB*/
  Serial.print("\t VEAB1_0:");
  Serial.print(VEAB_desired[0]);
  Serial.print("\t VEAB1_1:");
  Serial.print(VEAB_desired[1]);
  Serial.print("\t VEAB2_2:");
  Serial.print(VEAB_desired[2]);
  Serial.print("\t VEAB2_3:");
  Serial.print(VEAB_desired[3]);
  Serial.print("\t VEAB3_4:");
  Serial.print(VEAB_desired[4]);
  Serial.print("\t VEAB3_5:");
  Serial.print(VEAB_desired[5]);
  Serial.print("\t VEAB4_6:");
  Serial.print(VEAB_desired[6]);
  Serial.print("\t VEAB4_7:");
  Serial.print(VEAB_desired[7]);
  Serial.print("\t VEAB5_8:");
  Serial.print(VEAB_desired[8]);
  Serial.print("\t VEAB5_9:");
  Serial.print(VEAB_desired[9]);
  Serial.print("\t VEAB6_10:");
  Serial.print(VEAB_desired[10]);
  Serial.print("\t VEAB6_11:");
  Serial.println(VEAB_desired[11]);

}

/*------呼び出し関数----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
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

//VEAB1とVEAB2に与えるPWMの値を計算関数(停止モードにおける両ポートの値を基準に足し引きを行う) -1, -1, 1, -1, -1, -1
Result calculate_veab_Values(float outputPID, int i) {
  Result result;
  if(i == 0){
    result.veab_value1 = 136 + (outputPID / 2.0);  
    result.veab_value2 = 120 - (outputPID / 2.0);
  } else if(i == 1){
    result.veab_value1 = 145 + (outputPID / 2.0);  
    result.veab_value2 = 111 - (outputPID / 2.0);
  } else if(i == 2){
    result.veab_value1 = 127 + (outputPID / 2.0);  
    result.veab_value2 = 129 - (outputPID / 2.0);
  } else if(i == 3){
    result.veab_value1 = 126 + (outputPID / 2.0);  
    result.veab_value2 = 130 - (outputPID / 2.0);
  } else if(i == 4){
    result.veab_value1 = 138 + (outputPID / 2.0);  
    result.veab_value2 = 118 - (outputPID / 2.0);
  } else{
    result.veab_value1 = 152 + (outputPID / 2.0);  
    result.veab_value2 = 104 - (outputPID / 2.0);
  }

  result.veab_value1 = max(0, min(255, int(result.veab_value1)));
  result.veab_value2 = max(0, min(255, int(result.veab_value2)));

  return result;
}

//ローパスフィルタ関数
int lpf_function(int value, int previous_value, int initial_lpf, float coef_lpf){
  //ローパスフィルタの値初期化
  int filter_value = 0;

  //初回のみvalue値は同じものを使う
  if(initial_lpf == 0){
    previous_value = value;
  }

  //ローパスフィルタ計算
  filter_value = previous_value * coef_lpf + value * (1 - coef_lpf);

  return filter_value;
}