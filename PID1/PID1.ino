/*
【0.肩開閉、1.肩上下、2.上腕旋回、3.肘、4.前腕旋回、5.手首(小指)】用

2024/11/19
PID制御とBallistic Modeの実装
ROS2はなし

2024/11/21
ローパスフィルタ適用(POT, VEAB)

2024/12/3
ローパスフィルタ移動平均法適用

2024/12/9
PIDゲインチューニング完了

--注意--
使うTeensyによって、
1.PIDゲイン
2.Ballistic Modeのパラメータ値
3.圧力の正方向パラメータdirection
を変更！
*/

/*------ライブラリインクルード----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//キューのライブラリ
#include <queue>

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
int POT_realized[6] = {
  0, 0, 0, 0, 0, 0
};

/*VEABの値を初期化
int VEAB_realized[12] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};*/

/*---PID制御用-----------------------------------------------------------------------------------------*/
//PIDゲイン肘(仮)
const float kp[6] = {
  0.83, 2.1, 0.85, 0.2, 1.6, 0.6
};
const float ki[6] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};
const float kd[6] = {
  20.0, 25.0, 20.0, 2.0, 0.0, 0.0
};

//目標値の設定
int POT_desired[6] = {
  260, 548, 113, 220, 130, 500
};//{腕の閉223-482開, 腕の下344-619上, 上腕の旋回内95-605外, 肘の伸144-740曲, 前腕の旋回内111-962外, 小指側縮62-895伸}

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

/*------ローパスフィルタ-----------------------------------------------------------------------------------------*/
/*---RCフィルタ-----------------------------------------------------------------------------------------*/
//ローパスフィルタの係数　係数a=1/(2*pi*fc*dt + 1)   fc[Hz]:カットオフ周波数、dt[s]:サンプリング周期 
//カットオフ周波数:500Hz, サンプリング周期:0.000111s(9kHz)で設定
const float coef_lpf_veab = 0.75; //VEAB(開閉、上下、上腕) 
const float coef_lpf_veab_elbow = 0.75; //VEAB(肘の曲げ用)
const float coef_lpf_pot = 0.3;   //POT

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

/*---移動平均法-----------------------------------------------------------------------------------------*/
//ローパスフィルタ適用POT値格納構造体
struct Result_LPF {
  int POT0;
  int POT1;
  int POT2;
  int POT3;
  int POT4;
  int POT5;
};

//ローパスフィルタ1回目判定値
int LPF_count = 0;

//標本数
#define LPF_kosuu 1500

//キューを作成
std::queue<int> q0;
std::queue<int> q1;
std::queue<int> q2;
std::queue<int> q3;
std::queue<int> q4;
std::queue<int> q5;

//移動平均法での合計値と平均値格納
int pot_sum[6] = {0};
int POT[6] = {0};

/*------シリアル通信(ダウンサンプリング)-----------------------------------------------------------------------------------------*/
int serial_count = 0;
int serial_time = 91;

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
  analogWrite(PWM1_0, 136);
  analogWrite(PWM1_1, 120);
  /*ピン2,3*/
  analogWrite(PWM2_2, 170);
  analogWrite(PWM2_3, 86);
  /*ピン4,5*/
  analogWrite(PWM3_4, 127);
  analogWrite(PWM3_5, 129);
  /*ピン6,7*/
  analogWrite(PWM4_6, 131);
  analogWrite(PWM4_7, 125);
  /*ピン8,9*/
  analogWrite(PWM5_8, 127);
  analogWrite(PWM5_9, 129);
  /*ピン28,29*/
  analogWrite(PWM6_28, 132);
  analogWrite(PWM6_29, 124);

  //POT値をシリアルモニタに表示(初回)および誤差(前回分)の初期化
  while(digitalRead(Switch) == 1){

    //移動平均法ローパスフィルタを適用したPOT値をPOT_realizedに格納
    Result_LPF pot = Moving_LPF();
    POT_realized[0] = pot.POT0;
    POT_realized[1] = pot.POT1;
    POT_realized[2] = pot.POT2;
    POT_realized[3] = pot.POT3;
    POT_realized[4] = pot.POT4;
    POT_realized[5] = pot.POT5;

    //シリアルモニタに表示(1000回ループしたら)
    if(serial_count > serial_time){
      //POT
      /*Serial.print("POT1:");
      Serial.print(POT_realized[0]);
      Serial.print(",");
      Serial.println(10);*/

      /*Serial.print("\t POT2:");
      Serial.print(",");
      Serial.print(POT_realized[1]);
      Serial.print(",");
      Serial.println(10);*/

      /*Serial.print("\t POT3:");
      Serial.print(",");
      Serial.print(POT_realized[2]);
      Serial.print(",");
      Serial.println(10);*/

      /*Serial.print("\t POT4:");
      Serial.print(",");
      Serial.print(POT_realized[3]);
      Serial.print(",");
      Serial.println(10);*/

      /*Serial.print("\t POT5:");
      Serial.print(",");
      Serial.print(POT_realized[4]);
      Serial.print(",");
      Serial.println(10);*/

      /*Serial.print("\t POT6:");
      Serial.print(",");*/
      Serial.print(POT_realized[5]);
      Serial.print(",");
      Serial.println(10);

      //ループ回数の初期化
      serial_count = 0;
    }

    //誤差(前回分)の初期化
    for(int i = 0; i < 6; i++){
      previous_errors[i] = POT_desired[i] - POT_realized[i];
    }

    //ループの回数足し算
    serial_count += 1;

  }

  // 緊急ボタンの処理
  attachInterrupt(digitalPinToInterrupt(Switch_emergency), Emergency_function, FALLING);

}


/*------ループ処理----------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void loop() {
  /*
  unsigned long ts, te;
  ts = micros();*/

  /*------PID制御-----------------------------------------------------------------------------------------*/
  //移動平均法ローパスフィルタを適用したPOT値をPOT_realizedに格納
  Result_LPF pot = Moving_LPF();
  POT_realized[0] = pot.POT0;
  POT_realized[1] = pot.POT1;
  POT_realized[2] = pot.POT2;
  POT_realized[3] = pot.POT3;
  POT_realized[4] = pot.POT4;
  POT_realized[5] = pot.POT5;

  //RCローパスフィルタ適用(POT肘のみ)
  /*for(int i = 3; i < 4; i++){
    
    //ローパスフィルタ関数呼び出し
    pot_filter[i] = RC_LPF(POT_realized[i], previous_value_pot[i], initial_lpf_pot[i], coef_lpf_pot);

    initial_lpf_pot[i] += 1;

    //POT値に格納
    POT_realized[i] = pot_filter[i];

    //前回のPOT値に格納
    previous_value_pot[i] = pot_filter[i];
    
  }*/


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

  //RCローパスフィルタ適用(VEAB)
  for(int i = 0; i < 12; i++){

    //ローパスフィルタ関数呼び出し
    if((i == 6) || (i == 7)){
      veab_filter[i] = RC_LPF(VEAB_desired[i], previous_value_veab[i], initial_lpf_veab[i], coef_lpf_veab_elbow);
    }else {
      veab_filter[i] = RC_LPF(VEAB_desired[i], previous_value_veab[i], initial_lpf_veab[i], coef_lpf_veab);
    }

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
  /*ピン6,7
  analogWrite(PWM4_6, VEAB_desired[6]);
  analogWrite(PWM4_7, VEAB_desired[7]);*/
  /*ピン8,9
  analogWrite(PWM5_8, VEAB_desired[8]);
  analogWrite(PWM5_9, VEAB_desired[9]);*/
  /*ピン28,29*/
  analogWrite(PWM6_28, VEAB_desired[10]);
  analogWrite(PWM6_29, VEAB_desired[11]);

  //VEABの読み込み
  /*ピンA0,A1
  VEAB_realized[0] = analogRead(VEAB1_A0);
  VEAB_realized[1] = analogRead(VEAB1_A1);*/
  /*ピンA2,A3
  VEAB_realized[2] = analogRead(VEAB2_A2);
  VEAB_realized[3] = analogRead(VEAB2_A3);*/
  /*ピンA4,A5
  VEAB_realized[4] = analogRead(VEAB3_A4);
  VEAB_realized[5] = analogRead(VEAB3_A5);*/
  /*ピンA6,A7
  VEAB_realized[6] = analogRead(VEAB4_A6);
  VEAB_realized[7] = analogRead(VEAB4_A7);*/
  /*ピンA8,A9
  VEAB_realized[8] = analogRead(VEAB5_A8);
  VEAB_realized[9] = analogRead(VEAB5_A9);*/
  /*ピンA10,A11
  VEAB_realized[10] = analogRead(VEAB6_A10);
  VEAB_realized[11] = analogRead(VEAB6_A11);*/


  /*------シリアルモニタに表示-----------------------------------------------------------------------------------------*/
  if(serial_count > serial_time){
    /*POT*/
    /*Serial.print("POT1:");
    Serial.print(POT_realized[0]);
    Serial.print(",");
    Serial.println(11);*/

    /*Serial.print("\t POT2:");
    Serial.print(",");
    Serial.print(POT_realized[1]);
    Serial.print(",");
    Serial.println(11);*/

    /*Serial.print("\t POT3:");
    Serial.print(",");
    Serial.print(POT_realized[2]);
    Serial.print(",");
    Serial.println(11);*/

    /*Serial.print("\t POT4:");
    Serial.print(",");
    Serial.print(POT_realized[3]);
    Serial.print(",");
    Serial.print(11);*/

    /*Serial.print("\t POT5:");
    Serial.print(",");
    Serial.print(POT_realized[4]);
    Serial.print(",");
    Serial.println(11);*/

    /*Serial.print("\t POT6:");
    Serial.print(",");*/
    Serial.print(POT_realized[5]);
    Serial.print(",");
    Serial.println(11);


    /*VEAB*/
    /*Serial.print("\t VEAB1_0:");
    Serial.print(",");
    Serial.print(VEAB_desired[0]);
    //Serial.print("\t VEAB1_1:");
    Serial.print(",");
    Serial.println(VEAB_desired[1]);*/

    /*Serial.print("\t VEAB2_2:");
    Serial.print(",");
    Serial.print(VEAB_desired[2]);
    Serial.print("\t VEAB2_3:");
    Serial.print(",");
    Serial.print(VEAB_desired[3]);*/

    /*Serial.print("\t VEAB3_4:");
    Serial.print(",");
    Serial.print(VEAB_desired[4]);
    Serial.print("\t VEAB3_5:");
    Serial.print(",");
    Serial.print(VEAB_desired[5]);*/

    /*Serial.print("\t VEAB4_6:");
    Serial.print(",");
    Serial.print(VEAB_desired[6]);
    //Serial.print("\t VEAB4_7:");
    Serial.print(",");
    Serial.println(VEAB_desired[7]);*/

    /*Serial.print("\t VEAB5_8:");
    Serial.print(",");
    Serial.print(VEAB_desired[8]);
    Serial.print("\t VEAB5_9:");
    Serial.print(",");
    Serial.print(VEAB_desired[9]);*/

    /*Serial.print("\t VEAB6_10:");
    Serial.print(",");
    Serial.print(VEAB_desired[10]);
    Serial.print("\t VEAB6_11:");
    Serial.print(",");
    Serial.println(VEAB_desired[11]);*/
    
    //ループ回数の初期化
    serial_count = 0;

  }

  /*
  te = micros();
  Serial.print("\t Time:");
  Serial.println(te - ts); // μs*/

  //ループの回数足し算
  serial_count += 1;

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
    result.veab_value1 = 170 + (outputPID / 2.0);  
    result.veab_value2 = 86 - (outputPID / 2.0);
  } else if(i == 2){
    result.veab_value1 = 127 + (outputPID / 2.0);  
    result.veab_value2 = 129 - (outputPID / 2.0);
  } else if(i == 3){
    result.veab_value1 = 131 + (outputPID / 2.0);  
    result.veab_value2 = 125 - (outputPID / 2.0);
  } else if(i == 4){
    result.veab_value1 = 127 + (outputPID / 2.0);  
    result.veab_value2 = 129 - (outputPID / 2.0);
  } else{
    result.veab_value1 = 132 + (outputPID / 2.0);  
    result.veab_value2 = 124 - (outputPID / 2.0);
  }

  result.veab_value1 = max(0, min(255, int(result.veab_value1)));
  result.veab_value2 = max(0, min(255, int(result.veab_value2)));

  return result;
}

//ローパスフィルタ(RCフィルタ)
int RC_LPF(int value, int previous_value, int initial_lpf, float coef_lpf){
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

//ローパスフィルタ(移動平均法)
Result_LPF Moving_LPF() {
  Result_LPF lpf;

  //読み込んだ値potとキューの先頭の値pot_last初期化
  int pot[6] = {0};
  int pot_last[6] = {0};

  if(LPF_count == 0){
    for(int i = 0; i < LPF_kosuu; i++){
      //1自由度
      pot[0] = analogRead(POT1_A12); // アナログ値を1回だけ読み取る
      q0.push(pot[0]);               // キューに追加
      pot_sum[0] += pot[0];          // 合計を更新

      //2自由度
      pot[1] = analogRead(POT2_A13);
      q1.push(pot[1]);
      pot_sum[1] += pot[1];
      
      //3自由度
      pot[2] = analogRead(POT3_A14);
      q2.push(pot[2]);
      pot_sum[2] += pot[2];

      //4自由度
      pot[3] = analogRead(POT4_A15);
      q3.push(pot[3]);
      pot_sum[3] += pot[3];

      //5自由度
      pot[4] = analogRead(POT5_A16);
      q4.push(pot[4]);
      pot_sum[4] += pot[4];

      //6自由度
      pot[5] = analogRead(POT6_A17);
      q5.push(pot[5]);
      pot_sum[5] += pot[5];

    }

    // 初期の平均値を計算
    POT[0] = pot_sum[0] / LPF_kosuu;
    POT[1] = pot_sum[1] / LPF_kosuu;
    POT[2] = pot_sum[2] / LPF_kosuu;
    POT[3] = pot_sum[3] / LPF_kosuu;
    POT[4] = pot_sum[4] / LPF_kosuu;
    POT[5] = pot_sum[5] / LPF_kosuu;

  }else{
    //1自由度
    pot_last[0] = q0.front();        // 最初の値を取得
    q0.pop();                        // 最初の値を削除
    pot_sum[0] -= pot_last[0];       // 合計から削除した値を引く
    pot[0] = analogRead(POT1_A12);   // 新しいアナログ値を1回だけ読み取る
    q0.push(pot[0]);                 // キューに追加
    pot_sum[0] += pot[0];            // 合計に加算
    POT[0] = pot_sum[0] / LPF_kosuu; // 新しい平均値を計算

    //2自由度
    pot_last[1] = q1.front();        // 最初の値を取得
    q1.pop();                        // 最初の値を削除
    pot_sum[1] -= pot_last[1];       // 合計から削除した値を引く
    pot[1] = analogRead(POT2_A13);   // 新しいアナログ値を1回だけ読み取る
    q1.push(pot[1]);                 // キューに追加
    pot_sum[1] += pot[1];            // 合計に加算
    POT[1] = pot_sum[1] / LPF_kosuu; // 新しい平均値を計算

    //3自由度
    pot_last[2] = q2.front();        // 最初の値を取得
    q2.pop();                        // 最初の値を削除
    pot_sum[2] -= pot_last[2];       // 合計から削除した値を引く
    pot[2] = analogRead(POT3_A14);   // 新しいアナログ値を1回だけ読み取る
    q2.push(pot[2]);                 // キューに追加
    pot_sum[2] += pot[2];            // 合計に加算
    POT[2] = pot_sum[2] / LPF_kosuu; // 新しい平均値を計算

    //4自由度
    pot_last[3] = q3.front();        // 最初の値を取得
    q3.pop();                        // 最初の値を削除
    pot_sum[3] -= pot_last[3];       // 合計から削除した値を引く
    pot[3] = analogRead(POT4_A15);   // 新しいアナログ値を1回だけ読み取る
    q3.push(pot[3]);                 // キューに追加
    pot_sum[3] += pot[3];            // 合計に加算
    POT[3] = pot_sum[3] / LPF_kosuu; // 新しい平均値を計算

    //5自由度
    pot_last[4] = q4.front();        // 最初の値を取得
    q4.pop();                        // 最初の値を削除
    pot_sum[4] -= pot_last[4];       // 合計から削除した値を引く
    pot[4] = analogRead(POT5_A16);   // 新しいアナログ値を1回だけ読み取る
    q4.push(pot[4]);                 // キューに追加
    pot_sum[4] += pot[4];            // 合計に加算
    POT[4] = pot_sum[4] / LPF_kosuu; // 新しい平均値を計算

    //6自由度
    pot_last[5] = q5.front();        // 最初の値を取得
    q5.pop();                        // 最初の値を削除
    pot_sum[5] -= pot_last[5];       // 合計から削除した値を引く
    pot[5] = analogRead(POT6_A17);   // 新しいアナログ値を1回だけ読み取る
    q5.push(pot[5]);                 // キューに追加
    pot_sum[5] += pot[5];            // 合計に加算
    POT[5] = pot_sum[5] / LPF_kosuu; // 新しい平均値を計算

  }

  lpf.POT0 = POT[0];
  lpf.POT1 = POT[1];
  lpf.POT2 = POT[2];
  lpf.POT3 = POT[3];
  lpf.POT4 = POT[4];
  lpf.POT5 = POT[5];

  LPF_count = 1;

  return lpf;
}