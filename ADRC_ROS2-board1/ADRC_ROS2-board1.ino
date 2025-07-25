//=============================================================================================================================================
//------周期--------------------------------------------------------------------
//  制御周期(単位：ms)
#define CONTROL_PERIOD_MS 1

//  ROSのPublishする周期(単位：ms)
#define PUB_PERIOD_MS 10

//------LEDの定義ピン--------------------------------------------------------------------
#define LED 13

//------IP (Configure following IPs for your environment)--------------------------------------------------------------------
#define TEENSY_IP 192, 168, 1, 116 // IP that the teensy 4.1 will have
#define AGENT_IP 192, 168, 1, 162 // IP where a micro-ros agent waits

//------Topic names--------------------------------------------------------------------
#define SUB_TOPICNAME "/board_float/sub"
#define PUB_TOPICNAME "/board1/pub"

//------目標値の個数--------------------------------------------------------------------
#define POT_DESIRED 6

//------パラメータの個数--------------------------------------------------------------------
#define PARAMETER 0

//------微分値の個数--------------------------------------------------------------------
#define OMEGA 6

//------Publishするデータの個数--------------------------------------------------------------------
#define PUBLISH 6 + OMEGA //  ANALOG_IN_CH + OMEGA

//------Subscribeするデータの個数--------------------------------------------------------------------
#define SUBSCRIBE 6 + 1 + PARAMETER //  POT_DESIRED + 7自由度目の目標値 + PARAMETER

//------AD/DA conv channels--------------------------------------------------------------------
//  アナログ入力のピン数
//#define ANALOG_IN_CH 18
#define ANALOG_IN_CH 6

//  アナログ出力のピン数
#define ANALOG_OUT_CH 12

//  VEABのアナログ出力(14～25),POTのアナログ出力(26～41)
//const int ain_channels[ANALOG_IN_CH] = {14,15,16,17,18,19,20,21,22,23,24,25,26,27,38,39,40,41};
const int ain_channels[ANALOG_IN_CH] = {26,27,38,39,40,41};

//  PWMピンの定義
const int aout_channels[ANALOG_OUT_CH] = {0,1,2,3,4,5,6,7,8,9,28,29};

//=============================================================================================================================================

//------ライブラリのインクルード--------------------------------------------------------------------
//  Arduino機能のライブラリ
#include <Arduino.h>
//  MicroRosのライブラリ
#include <micro_ros_arduino.h>
//  TeensyThreadsのライブラリ
#include "TeensyThreads.h"

//  リングバッファのライブラリ
#include <CircularBuffer.hpp>

//  ROS2のおまじない
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//  使用するメッセージの型のライブラリ
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

//------マクロの定義--------------------------------------------------------------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
//  ROS2の関数fnが成功したかどうかを確認。成功したら何もせず次の処理に進み、失敗したらerror_loop()を呼び出しエラー状態にする

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
//  ROS2の関数fnが成功したかどうかを確認。成功したら何もせず次の処理に進み、失敗しても何もせずスルーする

//------オブジェクトの定義--------------------------------------------------------------------
//  使用するROS2メッセージの型
std_msgs__msg__UInt16MultiArray msg_pub;
std_msgs__msg__Float32MultiArray msg_sub;

//  ROS2のオブジェクトの定義
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//=============================================================================================================================================

//------グローバル変数の定義--------------------------------------------------------------------
//  subscribeメッセージの配列
volatile float sub[SUBSCRIBE];

//  publishメッセージの配列
volatile uint16_t pub[PUBLISH];

//  POT値
volatile uint16_t POT_realized[6] = {0, 0, 0, 0, 0, 0};

//  目標値の初期値{腕の閉190-394開, 腕の下287-534上, 上腕の旋回内87-500外, 肘の伸124-635曲, 前腕の旋回内98-900外, 小指側縮48-822伸}
/*No1
volatile uint16_t POT_desired[6] = {
  258, 300, 390, 500, 600, 500
};*/

/*No2, No3, No4*/
volatile uint16_t POT_desired[6] = {
  260, 400, 180, 500, 600, 500
};

//---ADRC--------------------------------------------------------------------
/*//  PDゲイン（単自由度）No1(2025/02/24)
const float kp[6] = {
  3000.0, 4000.0, 3200.0, 600.0, 2000.0, 900.0
};
const float kd[6] = {
  85.0, 180.0, 200.0, 40.0, 120.0, 20.0
};*/

/*//  PDゲイン（単自由度）No2(2025/02/26)
const float kp[6] = {
  1200.0, 1000.0, 2000.0, 400.0, 2000.0, 600.0
};
const float kd[6] = {
  65.0, 75.0, 220.0, 65.0, 120.0, 30.0
};*/

/*//  PDゲイン（単自由度）No3(2025/03/31)
const float kp[6] = {
  2000.0, 3000.0, 9000.0, 17000.0, 7000.0, 18000.0
};
const float kd[6] = {
  70.0, 120.0, 700.0, 2000.0, 400.0, 300.0
};*/

/*//  PDゲイン（単自由度）No4(2025/07/01)*/
const float kp[6] = {
  2500.0, 4000.0, 9000.0, 1400.0, 7000.0, 18000.0
};
const float kd[6] = {
  100.0, 200.0, 700.0, 110.0, 400.0, 300.0
};

/*//  PDゲイン（単自由度）調整用
const float kp[6] = {
  2500.0, 4000.0, 10000.0, 1400.0, 7000.0, 18000.0
};
const float kd[6] = {
  100.0, 200.0, 800.0, 110.0, 400.0, 300.0
};*/

//  オブザーバゲイン
//  オブザーバーの極(-λ₀の重根) 
/*No1,No2,No3
float lamda_0[6] = {300.0, 300.0, 300.0, 300.0, 300.0, 300.0};*/

/*No4*/
float lamda_0[6] = {400.0, 800.0, 300.0, 800.0, 300.0, 300.0};

/*調整用
float lamda_0[6] = {400.0, 800.0, 600.0, 800.0, 300.0, 300.0};*/

//  ゲイン
const float beta1[6] = {
  3 * lamda_0[0], 3 * lamda_0[1], 3 * lamda_0[2], 3 * lamda_0[3], 3 * lamda_0[4], 3 * lamda_0[5]
};
const float beta2[6] = {
  3 * lamda_0[0] * lamda_0[0], 3 * lamda_0[1] * lamda_0[1], 3 * lamda_0[2] * lamda_0[2], 3 * lamda_0[3] * lamda_0[3], 3 * lamda_0[4] * lamda_0[4], 3 * lamda_0[5] * lamda_0[5]
};
const float beta3[6] = {
  lamda_0[0] * lamda_0[0] * lamda_0[0], lamda_0[1] * lamda_0[1] * lamda_0[1], lamda_0[2] * lamda_0[2] * lamda_0[2], lamda_0[3] * lamda_0[3] * lamda_0[3], lamda_0[4] * lamda_0[4] * lamda_0[4], lamda_0[5] * lamda_0[5] * lamda_0[5]
};

//  ESO(拡張状態オブザーバ)
//  z1:角度の推定値
float dz1[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float z1[6] = {0, 0, 0, 0, 0, 0};
//  z2:角速度の推定値
float dz2[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float z2[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//  z3:外乱の推定値
float dz3[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float z3[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  制御入力の係数
/*No1,No2
float input_coef[6] = {10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0};*/

/*No3 POT0_4500 or 5500
float input_coef[6] = {4500.0, 8000.0, 20000.0, 70000.0, 20000.0, 40000.0};*/

/*No4*/
float input_coef[6] = {8000.0, 30000.0, 20000.0, 40000.0, 20000.0, 40000.0};

/*調整用
float input_coef[6] = {8000.0, 30000.0, 30000.0, 40000.0, 20000.0, 40000.0};*/

//  各自由度ごとの圧力の正方向とポテンショメータの正方向の対応を整理
const int direction[6] = {-1, -1, 1, -1, -1, -1};

//  各要素(自由度)の誤差
float errors[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  ADRC計算値
float outputADRC[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  ADRC値をアクチュエータの正方向に変換した値
float outputADRC_direct[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  ADRC PWM値
int ADRC_PWM[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//  初回のESO用時間変数
int T_start = 0;
int T_stop = 0;

//  初回のESO用
int eso_count = 0;

//---VEABへのPWM信号の出力--------------------------------------------------------------------
//  VEABへのPWM出力値用構造体
struct Result {
  float veab_value1;
  float veab_value2;
};

//  VEABへのPWM出力値
int VEAB_desired[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//---ローパスフィルタ--------------------------------------------------------------------
//--RCフィルタ--------------------------------------------------------------------
//  ローパスフィルタの係数  係数a=1/(2*pi*fc*dt + 1)   fc[Hz]:カットオフ周波数、dt[s]:サンプリング周期
//  カットオフ周波数:Hz, サンプリング周期:で設定
const float coef_lpf_veab = 0.0;   //  VEAB(カットオフ周波数150Hz)
const float coef_lpf_omega = 0.52;  //  角速度(カットオフ周波数150Hz)

//  ローパスフィルタの値保持変数
int veab_filter[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //  VEAB
float omega_filter[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};     //  角速度

//  ローパスフィルタ用前回の値保持変数
int previous_value_veab[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //  VEAB
float previous_value_omega[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};     //  角速度

//  ローパスフィルタ用初回判定
int initial_lpf_veab[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // VEAB
int initial_lpf_omega[6] = {0, 0, 0, 0, 0, 0};                    //  角速度

//--移動平均法--------------------------------------------------------------------
//  ローパスフィルタ適用POT値格納構造体
struct Result_LPF {
  int POT0;
  int POT1;
  int POT2;
  int POT3;
  int POT4;
  int POT5;
};

//  標本数
#define LPF_KOSUU 18

//  使用するチャンネル数
#define NUM_CHANNELS 6

//  CircularBufferを使用してリングバッファを作成
CircularBuffer<int, LPF_KOSUU> buffers[NUM_CHANNELS]; //  各チャンネル用のリングバッファ

//  移動平均法での合計値と平均値格納
long pot_sum[NUM_CHANNELS] = {0}; //  各チャンネルの値の合計
int POT[NUM_CHANNELS] = {0};      //  各チャンネルの移動平均値

//---スレッド間で共有リソースへのアクセスを制御するための排他制御（mutex: ミューテックス）を定義--------------------------------------------------------------------
Threads::Mutex adc_lock;

//---subscribeの初回判定--------------------------------------------------------------------
int sub_count = 0;

//---角速度計算--------------------------------------------------------------------
//  リングバッファの設定（データサイズは5点、6種類のデータに対応）
CircularBuffer<int, 5> omegaBuffers[6];

//  サンプリング間隔hを格納
float h = 0.001;   //  初期値を設定（秒単位）

//  角速度値
float derivatives[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//=============================================================================================================================================

//------関数の定義--------------------------------------------------------------------
//  MACアドレスを取得するための関数
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}

//  エラー関数
void error_loop(){
  while(1){
    //  エラー状態のときLED点滅
    digitalWrite(LED, !digitalRead(LED));
    delay(100);
  }
}

//  スレッドに追加した関数
void thread_callback() {
  while(1) {
    int t0 = millis();

    //====================================
    // write codes from here
    //====================================

    if(1) {
      //  スレッドセーフ(アクセスを阻止)
      Threads::Scope scope(adc_lock);

      //---データ取得--------------------------------------------------------------------
      //  移動平均法ローパスフィルタを適用したPOT値をPOT_realizedに格納
      Result_LPF pot = Moving_LPF();
      POT_realized[0] = pot.POT0;
      POT_realized[1] = pot.POT1;
      POT_realized[2] = pot.POT2;
      POT_realized[3] = pot.POT3;
      POT_realized[4] = pot.POT4;
      POT_realized[5] = pot.POT5;

      //  各ポテンショメータのデータをリングバッファ(角速度計算用)に追加
      for (int i = 0; i < OMEGA; i++) {
        omegaBuffers[i].unshift(POT_realized[i]);
      }

      //  各ポテンショメータの微分値を計算
      for (int i = 0; i < OMEGA; i++) {
        derivatives[i] = calculateDerivative(omegaBuffers[i]);
      }

      //  RCローパスフィルタ適用(角速度)
      for(int i = 0; i < OMEGA; i++){

        //  ローパスフィルタ関数呼び出し
        omega_filter[i] = RC_LPF_float(derivatives[i], previous_value_omega[i], initial_lpf_omega[i], coef_lpf_omega);

        initial_lpf_omega[i] = 1;

        //  微分値に格納
        derivatives[i] = omega_filter[i];

        //  前回の微分値に格納
        previous_value_omega[i] = omega_filter[i];
        
      }

      //---ROS2メッセージに格納--------------------------------------------------------------------
      //  publishメッセージの配列にPOT値を格納
      for(int i = 0; i < ANALOG_IN_CH; i++){
        pub[i] = POT_realized[i];
      }

      //  publishメッセージの配列に微分値(絶対値)を格納
      for (int i = 0; i < OMEGA; i++){
        pub[i+6] = abs(derivatives[i]);
      }

      //  subscribeしたメッセージを目標値に格納
      for(int i = 0; i < POT_DESIRED; i++){
        if(sub_count == 0){
          sub[i] = POT_desired[i];
        }

        POT_desired[i] = sub[i];
      }

    }

    //  拡張状態オブザーバの計算
    for(int i = 0; i < 6; i++){
      ESO(i);
    }

    //  ADRC PWM値計算
    for(int i = 0; i < 6; i++){
      ADRC(i);
    }

    //  RCローパスフィルタ適用(VEAB)
    for(int i = 0; i < ANALOG_OUT_CH; i++){

      //  ローパスフィルタ関数呼び出し
      veab_filter[i] = RC_LPF_int(ADRC_PWM[i], previous_value_veab[i], initial_lpf_veab[i], coef_lpf_veab);

      initial_lpf_veab[i] = 1;

      //  PWM値に格納
      ADRC_PWM[i] = veab_filter[i];

      //  前回のVEAB値に格納
      previous_value_veab[i] = veab_filter[i];
      
    }

    //  ADRC PWM値格納
    for(int i = 0; i < 6; i++){
      VEAB_desired[2*i] = ADRC_PWM[2*i];
      VEAB_desired[2*i+1] = ADRC_PWM[2*i+1];

    }

    //  シリアルモニタに表示
    SerialPrint_function();

    //------VEABへ出力--------------------------------------------------------------------
    /*ピン0,1*/
    analogWrite(aout_channels[0], VEAB_desired[0]);
    analogWrite(aout_channels[1], VEAB_desired[1]);
    /*ピン2,3*/
    analogWrite(aout_channels[2], VEAB_desired[2]);
    analogWrite(aout_channels[3], VEAB_desired[3]);
    /*ピン4,5*/
    analogWrite(aout_channels[4], VEAB_desired[4]);
    analogWrite(aout_channels[5], VEAB_desired[5]);
    /*ピン6,7*/
    analogWrite(aout_channels[6], VEAB_desired[6]);
    analogWrite(aout_channels[7], VEAB_desired[7]);
    /*ピン8,9*/
    analogWrite(aout_channels[8], VEAB_desired[8]);
    analogWrite(aout_channels[9], VEAB_desired[9]);
    /*ピン28,29*/
    analogWrite(aout_channels[10], VEAB_desired[10]);
    analogWrite(aout_channels[11], VEAB_desired[11]);

    //====================================
    // to here
    //====================================
    int t1 = millis();
    //Serial.println(t1-t0);

    //  制御周期を満たすように調整
    if( t1-t0 <= CONTROL_PERIOD_MS ){
      //  spare cpu slices to other threads for the remaining time
      threads.delay(CONTROL_PERIOD_MS - (t1-t0));
    } else {
      //  the designated period is violated
      error_loop();
    }

  }
}

//  Publish関数（ROS2）
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  //  引数last_call_timeは使わない
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    //  Publlishするメッセージmsg_pubに格納
    for (size_t i = 0; i < PUBLISH; i++) {
      msg_pub.data.data[i] = pub[i];
    }
    //  メッセージをトピックに送信
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
  }
}

//  Subscribe関数（ROS2）
void subscription_callback(const void * msgin)
{
  //  受け取ったメッセージmsginをstd_msgs__msg__Float32MultiArray型にキャスト
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  //  Subscribeしたメッセージを格納
  for (size_t i = 0; i < SUBSCRIBE; i++) {
    sub[i] = msg->data.data[i];
  }

  //  Subscribe関数の初回判定
  sub_count = 1;
  
}

//  ESO関数
void ESO(int index){
  //  1回目のみ実現値を推定値に格納
  if(eso_count == 0){
    for(int i = 0; i < 6; i++){
      z1[i] = (float)POT_realized[i];

      eso_count = 1;
    }
  }

  //  角度の推定
  dz1[index] = z2[index] + beta1[index] * ((float)POT_realized[index] - z1[index]);
  //  角速度の推定
  dz2[index] = z3[index] + beta2[index] * ((float)POT_realized[index] - z1[index]) + input_coef[index] * outputADRC[index];
  //  外乱（動特性）の推定
  dz3[index] = beta3[index] * ((float)POT_realized[index] - z1[index]);

  //  角度
  z1[index] += dz1[index] * 0.001;
  //  角速度
  z2[index] += dz2[index] * 0.001;
  //  外乱
  z3[index] += dz3[index] * 0.001;

}

//  ADRC制御関数
void ADRC(int index){

  //  誤差計算
  errors[index] = (float)POT_desired[index] - z1[index];

  //  ADRC計算
  outputADRC[index] = (-z3[index] + kp[index] * errors[index] - kd[index] * z2[index]) / input_coef[index];

  //  アンドロイドのアクチュエータの要件（ポテンショメータの正方向と入力の正方向を一致させる）
  outputADRC_direct[index] = outputADRC[index] * direction[index];

  //  VEAB1とVEAB2に与えるPWMの値を計算し格納
  Result veab = calculate_veab_Values(outputADRC_direct[index], index);
  ADRC_PWM[2*index] = veab.veab_value1;   //0, 2, 4, 6, 8, 10ピンへ
  ADRC_PWM[2*index+1] = veab.veab_value2; //1, 3, 5, 7, 9, 11ピンへ

}

//  VEAB1とVEAB2に与えるPWMの値の計算関数(アクチュエータの両ポートの圧力が釣り合うPWN値を基準に足し引きを行う)
Result calculate_veab_Values(float outputADRC_derect, int i) {
  Result result;
  if(i == 0){
    result.veab_value1 = 141 + (outputADRC_derect / 2.0);  
    result.veab_value2 = 115 - (outputADRC_derect / 2.0);
  } else if(i == 1){
    result.veab_value1 = 126 + (outputADRC_derect / 2.0);  
    result.veab_value2 = 130 - (outputADRC_derect / 2.0);
  } else if(i == 2){
    result.veab_value1 = 127 + (outputADRC_derect / 2.0);  
    result.veab_value2 = 129 - (outputADRC_derect / 2.0);
  } else if(i == 3){
    result.veab_value1 = 136 + (outputADRC_derect / 2.0);  
    result.veab_value2 = 120 - (outputADRC_derect / 2.0);
  } else if(i == 4){
    result.veab_value1 = 127 + (outputADRC_derect / 2.0);  
    result.veab_value2 = 129 - (outputADRC_derect / 2.0);
  } else{
    result.veab_value1 = 132 + (outputADRC_derect / 2.0);  
    result.veab_value2 = 124 - (outputADRC_derect / 2.0);
  }

  result.veab_value1 = max(0, min(255, int(result.veab_value1)));
  result.veab_value2 = max(0, min(255, int(result.veab_value2)));

  return result;
}

//  微分値計算関数
float calculateDerivative(CircularBuffer<int, 5> &buffer) {
  //  バッファに5点のデータがある場合のみ計算
  if (buffer.size() < 5) {
    return 0; //  データが不足している場合は0を返す
  }

  //  リングバッファからデータを取得し、float型に変換
  float f0 = static_cast<float>(buffer[0]);
  float f1 = static_cast<float>(buffer[1]);
  float f2 = static_cast<float>(buffer[2]);
  float f3 = static_cast<float>(buffer[3]);
  float f4 = static_cast<float>(buffer[4]);

  //  4階の後退差分で微分値を計算
  return (25 * f0 - 48 * f1 + 36 * f2 - 16 * f3 + 3 * f4) / (12 * h);
}

//  ローパスフィルタ(RCフィルタ)関数※int型
int RC_LPF_int(int value, int previous_value, int initial_lpf, float coef_lpf){
  //  ローパスフィルタの値初期化
  int filter_value = 0;

  //  初回のみvalue値は同じものを使う
  if(initial_lpf == 0){
    previous_value = value;
  }

  //  ローパスフィルタ計算
  filter_value = previous_value * coef_lpf + value * (1 - coef_lpf);

  return filter_value;
}

//  ローパスフィルタ(RCフィルタ)関数※float型
float RC_LPF_float(float value, float previous_value, int initial_lpf, float coef_lpf){
  //ローパスフィルタの値初期化
  float filter_value = 0;

  //初回のみvalue値は同じものを使う
  if(initial_lpf == 0){
    previous_value = value;
  }

  //ローパスフィルタ計算
  filter_value = previous_value * coef_lpf + value * (1 - coef_lpf);

  return filter_value;
}

//  ローパスフィルタ(移動平均法)関数
Result_LPF Moving_LPF() {
  Result_LPF lpf;

  //  各チャンネルの処理
  for (int i = 0; i < NUM_CHANNELS; i++) {
    int new_value = analogRead(ain_channels[i]);  //  新しいアナログ値を読み取る

    //  リングバッファが満杯の場合、古い値を取り出して合計から引く
    if (buffers[i].isFull()) {
      pot_sum[i] -= buffers[i].first(); //  最古の値を引く
    }

    //  新しい値をバッファに追加し、合計を更新
    buffers[i].push(new_value);
    pot_sum[i] += new_value;

    //  移動平均を計算
    POT[i] = pot_sum[i] / buffers[i].size();  //  バッファ内の現在の要素数で割る
  }

  lpf.POT0 = POT[0];
  lpf.POT1 = POT[1];
  lpf.POT2 = POT[2];
  lpf.POT3 = POT[3];
  lpf.POT4 = POT[4];
  lpf.POT5 = POT[5];

  return lpf;
}

//  シリアルモニタ関数
void SerialPrint_function(){
  /**/
  Serial.print(POT_realized[0]);
  Serial.print(",");
  Serial.print(z1[0]);
  Serial.print(",");
  Serial.print(dz1[0]);
  Serial.print(",");
  Serial.print(derivatives[0]);
  Serial.print(",");
  Serial.print(z2[0]);
  Serial.print(",");
  Serial.print(dz2[0]);
  Serial.print(",");
  Serial.print(z3[0]);
  Serial.print(",");
  Serial.print(dz3[0]);
  Serial.print(",");
  Serial.print(outputADRC[0]);
  Serial.print(",");
  Serial.print(POT_desired[0]);
  /**/
  Serial.print(",");
  Serial.print(POT_realized[1]);
  Serial.print(",");
  Serial.print(z1[1]);
  Serial.print(",");
  Serial.print(dz1[1]);
  Serial.print(",");
  Serial.print(derivatives[1]);
  Serial.print(",");
  Serial.print(z2[1]);
  Serial.print(",");
  Serial.print(dz2[1]);
  Serial.print(",");
  Serial.print(z3[1]);
  Serial.print(",");
  Serial.print(dz3[1]);
  Serial.print(",");
  Serial.print(outputADRC[1]);
  Serial.print(",");
  Serial.print(POT_desired[1]);
  /**/
  Serial.print(",");
  Serial.print(POT_realized[2]);
  Serial.print(",");
  Serial.print(z1[2]);
  Serial.print(",");
  Serial.print(dz1[2]);
  Serial.print(",");
  Serial.print(derivatives[2]);
  Serial.print(",");
  Serial.print(z2[2]);
  Serial.print(",");
  Serial.print(dz2[2]);
  Serial.print(",");
  Serial.print(z3[2]);
  Serial.print(",");
  Serial.print(dz3[2]);
  Serial.print(",");
  Serial.print(outputADRC[2]);
  Serial.print(",");
  Serial.print(POT_desired[2]);
  /**/
  Serial.print(",");
  Serial.print(POT_realized[3]);
  Serial.print(",");
  Serial.print(z1[3]);
  Serial.print(",");
  Serial.print(dz1[3]);
  Serial.print(",");
  Serial.print(derivatives[3]);
  Serial.print(",");
  Serial.print(z2[3]);
  Serial.print(",");
  Serial.print(dz2[3]);
  Serial.print(",");
  Serial.print(z3[3]);
  Serial.print(",");
  Serial.print(dz3[3]);
  Serial.print(",");
  Serial.print(outputADRC[3]);
  Serial.print(",");
  Serial.print(POT_desired[3]);
  /**/
  Serial.print(",");
  Serial.print(POT_realized[4]);
  Serial.print(",");
  Serial.print(z1[4]);
  Serial.print(",");
  Serial.print(dz1[4]);
  Serial.print(",");
  Serial.print(derivatives[4]);
  Serial.print(",");
  Serial.print(z2[4]);
  Serial.print(",");
  Serial.print(dz2[4]);
  Serial.print(",");
  Serial.print(z3[4]);
  Serial.print(",");
  Serial.print(dz3[4]);
  Serial.print(",");
  Serial.print(outputADRC[4]);
  Serial.print(",");
  Serial.print(POT_desired[4]);
  /**/
  Serial.print(",");
  Serial.print(POT_realized[5]);
  Serial.print(",");
  Serial.print(z1[5]);
  Serial.print(",");
  Serial.print(dz1[5]);
  Serial.print(",");
  Serial.print(derivatives[5]);
  Serial.print(",");
  Serial.print(z2[5]);
  Serial.print(",");
  Serial.print(dz2[5]);
  Serial.print(",");
  Serial.print(z3[5]);
  Serial.print(",");
  Serial.print(dz3[5]);
  Serial.print(",");
  Serial.print(outputADRC[5]);
  Serial.print(",");
  Serial.println(POT_desired[5]);
}

//=============================================================================================================================================

void setup() {
  //  TeensyのMACアドレスを取得
  byte teensy_mac[6];
  get_teensy_mac(teensy_mac);

  //  IPアドレスの設定
  IPAddress teensy_ip(TEENSY_IP); //  Teensy自身のIPアドレスを設定
  IPAddress agent_ip(AGENT_IP);   //  micro-ROS エージェントの IP アドレスを設定

  //   Teensyをmicro-ROSエージェントに接続するためのEthernet UDP設定(引数：MACアドレス, TeensyのIP, micro-ROSエージェントのIP, ポート番号)
  set_microros_native_ethernet_udp_transports(teensy_mac, teensy_ip, agent_ip, 9999);

  //  シリアル通信を初期化する。ボーレートは9600bps（デバック時に用いる。シリアルモニタを開いてから書き込みを行う）
  Serial.begin(9600);

  //  configure LED pin
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  //  configure PWM/AD pins
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    pinMode(aout_channels[i], OUTPUT);
  }
  for (size_t i = 0; i < ANALOG_IN_CH; i++) {
    pinMode(ain_channels[i], INPUT);
  }

  //  PWM周波数変更
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    analogWriteFrequency(aout_channels[i], 50000);
  }

  //------allocate message variables（pubもsubも1行目の右辺のみ変更可。その他はコピペ）--------------------------------------------------------------------
  //  1.メッセージ変数の初期化段階（メモリを確保し、デフォルト状態に設定）
  //  メッセージ変数msg_pubに対して、メモリの確保と初期化
  msg_pub.data.capacity = PUBLISH; //  data配列の最大要素数=メッセージの要素数
  msg_pub.data.size = 0;  //　メッセージ送信時のデータの初期化の保証
  msg_pub.data.data = (uint16_t*)malloc(msg_pub.data.capacity * sizeof(uint16_t));  //　data配列に必要なメモリを動的に確保
  msg_pub.layout.dim.capacity = 1;  //  1-dimentional array: vector（次元の数）
  msg_pub.layout.dim.size = 0;  //  配列の次元情報を初期化
  msg_pub.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_pub.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension)); //  MultiArrayDimension のためのメモリを確保
  for(size_t i = 0; i < msg_pub.layout.dim.capacity; i++){  //  ラベルフィールドの設定(コピペ)
    msg_pub.layout.dim.data[i].label.capacity = 20;
    msg_pub.layout.dim.data[i].label.size = 0;
    msg_pub.layout.dim.data[i].label.data = (char*) malloc(msg_pub.layout.dim.data[i].label.capacity * sizeof(char));
  }
  
  //  メッセージ変数msg_subに対して、メモリの確保と初期化
  msg_sub.data.capacity = SUBSCRIBE;  //  data配列の最大要素数=メッセージの要素数
  msg_sub.data.size = 0;  //　初期状態でメッセージを受け取ることを保証（初期化）
  msg_sub.data.data = (float*)malloc(msg_sub.data.capacity * sizeof(float));  //　data配列に必要なメモリを動的に確保
  msg_sub.layout.dim.capacity = 1;  //  1-dimentional array: vector
  msg_sub.layout.dim.size = 0;  //　配列の次元情報を初期化
  msg_sub.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_sub.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension)); //  MultiArrayDimension のためのメモリを確保
  for(size_t i = 0; i < msg_sub.layout.dim.capacity; i++){ //  ラベルフィールドの設定(コピペ)
    msg_sub.layout.dim.data[i].label.capacity = 20;
    msg_sub.layout.dim.data[i].label.size = 0;
    msg_sub.layout.dim.data[i].label.data = (char*) malloc(msg_sub.layout.dim.data[i].label.capacity * sizeof(char));
  }

  //------initialize message variables--------------------------------------------------------------------
  //  2.メッセージ変数の設定段階（送信可能な状態に設定）
  msg_pub.data.size = PUBLISH; //  メッセージの要素数を設定（データの実際の使用サイズ）
  msg_pub.layout.dim.size = 1;  //  メッセージのデータの次元数を設定(ex, 1 = 1次元の配列(ベクトル))
  msg_pub.layout.dim.data[0].label.size = strlen(msg_pub.layout.dim.data[0].label.data);  //labelの長さを設定
  msg_pub.layout.dim.data[0].size = PUBLISH; //  publishするメッセージの次元のサイズを設定
  msg_pub.layout.dim.data[0].stride = PUBLISH; //　publishするメッセージの更新ストライドを設定
  //  メッセージ変数を初期化
  for (size_t i = 0; i < PUBLISH; i++) {
    pub[i] = 0; //  pub配列（publishメッセージの値）の初期化
    msg_pub.data.data[i] = pub[i];  //  メッセージのデータ部分に初期化された値を格納
  }
  for (size_t i = 0; i < SUBSCRIBE; i++) {
    msg_sub.data.data[i] = 255.0; //  sub配列（subscribeメッセージの値）の初期化
    sub[i] = msg_sub.data.data[i]; //  メッセージデータをsub配列にコピー
    
  }

  delay(2000);

  //****************************************
  // micro-ROS functions setup
  //****************************************

  //  ①create default allocator（メモリ管理のためのアロケータ（メモリ確保用の管理機構）を取得）
  allocator = rcl_get_default_allocator();

  //  ②create init_options（サポートオプションを設定。引数もコピペでよい）
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //  ③create node（引数：(初期化するノード, ノードの名前, ノード空間の名前, サポート構造体)
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //------create subscriber（subscriberの初期化）--------------------------------------------------------------------
  RCCHECK(rclc_subscription_init_default(
    &subscriber,  //  subscriberの構造体を指定
    &node,  //  subscriberが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  subscriberが受け取るメッセージの型の定義(std_msgs/msg/UInt16MultiArray 型)
    SUB_TOPICNAME));  //  subscribeするトピックの名前

  //------create publisher--------------------------------------------------------------------
  RCCHECK(rclc_publisher_init_default(
    &publisher, //  publisherの構造体を指定
    &node,  //  publisherが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/UInt16MultiArray 型)
    PUB_TOPICNAME));  //  publishするトピックの名前

  //------create timer（timerを利用して定期的に実行するtimer_callback関数を設定）--------------------------------------------------------------------
  const unsigned int timer_timeout = PUB_PERIOD_MS;
  RCCHECK(rclc_timer_init_default(
    &timer, //  タイマー構造体
    &support, //  サポート構造体
    RCL_MS_TO_NS(timer_timeout),  //  周期を指定
    timer_callback)); //  コールバック関数の指定

  //------create executor--------------------------------------------------------------------
  //  Executor（実行管理エンジン）を初期化（引数：(初期化するExecutor構造体, Executorが利用するROS2のコンテキスト, Executorが管理するハンドル（ex. subscribe,timer）の数, メモリ割り当て用のカスタムアロケータ)）
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  //  SubscriberをExecutorに追加（引数：(初期化済みのExecutor, 追加するsubscriber構造体, subscriberが受信したメッセージ変数, subscriber用コールバック関数, 新しいデータが届いたときのみコールバックを実行する動作モード)）
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  //  timerをExecutorに追加（引数：(初期化済みのExecutor, 追加するtimer構造体)）
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //======setup関数内での実行処理==============================
  //  VEABの初期化
  /*ピン0,1*/
  analogWrite(aout_channels[0], 141);
  analogWrite(aout_channels[1], 115);
  /*ピン2,3*/
  analogWrite(aout_channels[2], 126);
  analogWrite(aout_channels[3], 130);
  /*ピン4,5*/
  analogWrite(aout_channels[4], 127);
  analogWrite(aout_channels[5], 129);
  /*ピン6,7*/
  analogWrite(aout_channels[6], 136);
  analogWrite(aout_channels[7], 120);
  /*ピン8,9*/
  analogWrite(aout_channels[8], 127);
  analogWrite(aout_channels[9], 129);
  /*ピン28,29*/
  analogWrite(aout_channels[10], 132);
  analogWrite(aout_channels[11], 124);

  delay(5000);

  //  移動平均法1回目の処理(ポテンショメータ)
  for (int i = 0; i < LPF_KOSUU; i++){
    Moving_LPF();
  }

  //  角速度計算のためポテンショメータの各データをリングバッファに追加(4階後退差分)
  for(int i = 0; i < 5; i++){
    Result_LPF pot = Moving_LPF();
    POT_realized[0] = pot.POT0;
    POT_realized[1] = pot.POT1;
    POT_realized[2] = pot.POT2;
    POT_realized[3] = pot.POT3;
    POT_realized[4] = pot.POT4;
    POT_realized[5] = pot.POT5;

    for(int i = 0; i < OMEGA; i++){
      omegaBuffers[i].unshift(POT_realized[i]);
    }

  }

  //==========================================================

  //  turn off LED
  digitalWrite(LED, LOW);

  //------run the control thread（新しいスレッドの作成。thread_cakkback関数が繰り返される）--------------------------------------------------------------------
  threads.addThread(thread_callback);

}

void loop() {

  //  spin the ros tasks（Executorを実行）
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
