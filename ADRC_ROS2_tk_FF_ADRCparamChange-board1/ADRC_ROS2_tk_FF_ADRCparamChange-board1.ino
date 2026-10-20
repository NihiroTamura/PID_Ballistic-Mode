//===調整必要パラメータ(P)===
/*
1. 目標値の初期値(初期位置の指定) POT_desired[]:142行目
2. アクチュエータと制御入力の正負方向対応係数 direction[]:159行目
3. PDゲイン: kp_??[], kd_??[]:197~207行目(Pゲイン), 211~222行目(Dゲイン)
※PDゲインの構造体が持つ要素数 kp_row_sizes[], kd_row_sizes[]:235行目(Pゲイン), 239行目(Dゲイン)
4. 制御入力係数 input_coef_??[]:245~256行目
※制御入力係数の構造体が持つ要素数 input_coef_row_sizes[]:265行目
5. z3外乱値係数 z3_??[]:271~282行目
※z3外乱値係数の構造体が持つ要素数 z3_row_sizes[]:291行目
6. オブザーバーゲインの極 lamda_0:302行目
7. 非対角要素のオブザーバーゲイン beta1?[], beta2?[], beta3?[]:312~323行目(beta1?[]), 326~337行目(beta2?[]), 340~351行目(beta3?[])
※オブザーバーゲインの構造体が持つ要素数 beta_row_sizes[]:362~364行目
8. calculate_veab_Values関数のPWM基準値 result.veab_value? = "〇"± (outputADRC_direct / 2.0):806~832行目
*/
//=============================================================================================================================================

//===ADRCパラメータの変更・状態量のpublish（GUI_android_tk_ADRCparamChg.py / GUI_android_tk.py 用）===
/*
・kp, kd, input_coef, lamda_0 を実行中に GUI から変更できる
    受信: SUB_TOPICNAME3 (Float32MultiArray, 25要素)
      [0]           : seq（送信番号。1以上の整数。GUIが応答の確認に使う）
      [1+4*i+0..3]  : 自由度 i（board内の番号 0~5）の kp, kd, input_coef, lamda_0
      NaN の要素は「変更しない」。1つでも不正な要素（要素数の違い・inf・範囲外・パラメータを変更できない自由度への値）
      があればメッセージ全体を反映しない（一部だけ反映することはしない）。範囲は KP_MIN~LAMDA_0_MAX を参照。
      反映は制御スレッドの周期の先頭で、自由度ごとに4つまとめて行う（lamda_0 を変えたらオブザーバーゲインを再計算）。
      ESOの状態が NaN/inf になっている自由度（input_coef=0 で動かしていた自由度）は、反映時にESOの状態を初期化する。
      反映した値は .ino を書き換えないので、Teensy を再起動すると .ino の初期値に戻る。通信が切れても最後に反映した値のまま。
・設定中のパラメータを publish
    送信: PUB_TOPICNAME7 (Float32MultiArray, 50要素, 200msごと + 受信を処理した直後)
      [0]            : 最後に処理した seq（起動後に未受信なら0）
      [1]            : 結果（1:反映した, 0:起動後未受信, -1:要素数が違う, -2:seqが不正, -3:inf, -4:範囲外, -5:変更できない自由度への値）
      [2+4*i+0..3]   : 自由度 i の現在の kp, kd, input_coef, lamda_0
      [26+4*i+0..3]  : 自由度 i の .ino の初期値（起動時の値）
      パラメータを変更できない自由度（i >= ADRC_PARAM_DOF）は NaN
・状態量を publish（PUB_PERIOD_MS ごと, Float32MultiArray, 6要素。要素 i = board内の自由度 i で、PUB_TOPICNAME3(z3) と同じ並び）
    PUB_TOPICNAME4: z1（角度の推定値）
    PUB_TOPICNAME5: derivatives（角速度。RCローパスフィルタ後の値）
    PUB_TOPICNAME6: z2（角速度の推定値）
*/
//=============================================================================================================================================
//------周期--------------------------------------------------------------------
//  制御周期(単位：ms)
#define CONTROL_PERIOD_MS 1

//  ROSのPublishする周期(単位：ms)
#define PUB_PERIOD_MS 10

//------LEDの定義ピン--------------------------------------------------------------------
#define LED 13

//------IP (Configure following IPs for your environment)--------------------------------------------------------------------
#define TEENSY_IP 192, 168, 1, 131 // IP that the teensy 4.1 will have ※teensyのIPは192.168.[0-3].[0-255]
#define AGENT_IP 192, 168, 1, 217 // IP where a micro-ros agent waits

//------Topic names--------------------------------------------------------------------
#define SUB_TOPICNAME "/board_android_float/sub"
#define SUB_TOPICNAME2 "/board1_FFparam_float/sub"
#define PUB_TOPICNAME "/board1_tk/pub"
#define PUB_TOPICNAME2 "/board1_tk_PWM_float/pub"
#define PUB_TOPICNAME3 "/board1_tk_z3_float/pub"
#define SUB_TOPICNAME3 "/board1_tk_ADRCparam_float/sub"
#define PUB_TOPICNAME4 "/board1_tk_z1_float/pub"
#define PUB_TOPICNAME5 "/board1_tk_derivatives_float/pub"
#define PUB_TOPICNAME6 "/board1_tk_z2_float/pub"
#define PUB_TOPICNAME7 "/board1_tk_ADRCparam_float/pub"

//------目標値の個数--------------------------------------------------------------------
#define POT_DESIRED 6

//------パラメータの個数--------------------------------------------------------------------
#define PARAMETER 0

//------微分値の個数--------------------------------------------------------------------
#define OMEGA 6

//------Publishするデータの個数--------------------------------------------------------------------
#define PUBLISH 6 + POT_DESIRED //  ANALOG_IN_CH + POT_DESIRED
#define PUBLISH2 6 //  自由度分の制御入力の数
#define PUBLISH3 6 //  自由度分の外乱の数
#define PUBLISH4 6 //  自由度分の角度推定値z1の数
#define PUBLISH5 6 //  自由度分の角速度derivativesの数
#define PUBLISH6 6 //  自由度分の角速度推定値z2の数
#define PUBLISH7 50 //  [処理したseq, 結果] + (kp, kd, input_coef, lamda_0) × 6自由度 × (現在値, 初期値)

//------Subscribeするデータの個数--------------------------------------------------------------------
#define SUBSCRIBE 6 + 20 + PARAMETER //  POT_DESIRED + 7~26自由度目の目標値 + PARAMETER
#define SUBSCRIBE2 36 //  (FFパラメータa,b,c,d,e + FF制御入力時間T) × 6自由度
#define SUBSCRIBE3 25 //  [seq] + (kp, kd, input_coef, lamda_0) × 6自由度
#define SUBSCRIBE3_CAPACITY 32 //  受信メッセージの領域（要素数が違うメッセージも受け取って拒否できるよう SUBSCRIBE3 より大きくする）

//------ADRCパラメータの変更--------------------------------------------------------------------
//  パラメータを変更できる自由度の数（board内の自由度 0~ADRC_PARAM_DOF-1）
#define ADRC_PARAM_DOF 6

//  受け付けるパラメータの範囲（GUI側 adrc_param_common.py の PARAM_LIMITS と同じにすること）
#define KP_MIN 0.0
#define KP_MAX 100000.0
#define KD_MIN 0.0
#define KD_MAX 10000.0
#define INPUT_COEF_MIN 1000.0
#define INPUT_COEF_MAX 1000000.0
#define LAMDA_0_MIN 1.0       //  ESOは dt=0.001 の前進オイラーなので lamda_0*dt < 2 が安定条件。余裕をみて1000まで
#define LAMDA_0_MAX 1000.0

//  設定中のパラメータをpublishする間隔（timer_callbackの回数。PUB_PERIOD_MS × 20 = 200ms）
#define PARAM_PUB_EVERY 20

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
#include <std_msgs/msg/int16_multi_array.h>
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
std_msgs__msg__Float32MultiArray msg_pub2;
std_msgs__msg__Float32MultiArray msg_pub3;
std_msgs__msg__Float32MultiArray msg_sub;
std_msgs__msg__Float32MultiArray msg_sub2;
std_msgs__msg__Float32MultiArray msg_pub4;
std_msgs__msg__Float32MultiArray msg_pub5;
std_msgs__msg__Float32MultiArray msg_pub6;
std_msgs__msg__Float32MultiArray msg_pub7;
std_msgs__msg__Float32MultiArray msg_sub3;

//  ROS2のオブジェクトの定義
rcl_subscription_t subscriber;
rcl_subscription_t subscriber2;
rcl_subscription_t subscriber3;
rcl_publisher_t publisher;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;
rcl_publisher_t publisher4;
rcl_publisher_t publisher5;
rcl_publisher_t publisher6;
rcl_publisher_t publisher7;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_init_options_t init_options;

//  ROS2 DOMAIN ID
size_t domain_id = 16;

//=============================================================================================================================================

//------グローバル変数の定義--------------------------------------------------------------------
//  subscribeメッセージの配列
volatile float sub[SUBSCRIBE];
volatile float sub2[SUBSCRIBE2];

//  publishメッセージの配列
volatile uint16_t pub[PUBLISH];
volatile float pub2[PUBLISH2];
volatile float pub3[PUBLISH3];
volatile float pub4[PUBLISH4];
volatile float pub5[PUBLISH5];
volatile float pub6[PUBLISH6];

//  POT値(POT No.[44, 43, 42, 41, 40, 39])
volatile float POT_realized[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//(P)  目標値の初期値(POT No.[44, 43, 42, 41, 40, 39])
/*No1*/
volatile float POT_desired[6] = {
  500.0, 200.0, 500.0, 300.0, 170.0, 300.0
};

//---ADRC--------------------------------------------------------------------
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

//(P)  各自由度ごとの圧力の正方向とポテンショメータの正方向の対応を整理
//  (POT No.[44, 43, 42, 41, 40, 39]) 
const int direction[6] = {1, -1, -1, -1, 1, 1};

//  各要素(自由度)の誤差
float errors[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  ADRC計算値
float outputADRC[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  ADRC値をアクチュエータの正方向に変換した値
float outputADRC_direct[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  ADRC PWM値
int ADRC_PWM[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//  初回のESO用
int eso_count = 0;

//---VEABへのPWM信号の出力--------------------------------------------------------------------
//  VEABへのPWM出力値用構造体
struct Result {
  float veab_value1;
  float veab_value2;
};

//  VEABへのPWM出力値(POT No.[44a, 44b, 43a, 43b, 42a, 42b, 41a, 41b, 40a, 40b, 39a, 39b])
int VEAB_desired[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//---MIMO--------------------------------------------------------------------
//  スパース配列構造体
struct SparseElement {
  int row;
  int col;
  float value;
};

//--PD制御入力値関連--------------------------------------------------------------------
//  (POT No.[44, 43, 42, 41, 40, 39])
//(P) -Pゲイン（各自由度　最大6要素）--------------------------------------------------------------------
/*//  自由度1*/
SparseElement kp_11[] = { {0, 0, 2200.0} };
//  自由度2
SparseElement kp_12[] = { {1, 1, 12000.0} };
//  自由度3
SparseElement kp_13[] = { {2, 2, 8000.0} };
//  自由度4
SparseElement kp_14[] = { {3, 3, 9000.0} };
//  自由度5
SparseElement kp_15[] = { {4, 4, 6000.0} };
//  自由度6
SparseElement kp_16[] = { {5, 5, 6000.0} };

//(P) -Dゲイン（各自由度　最大6要素）--------------------------------------------------------------------
/*//  自由度1*/
SparseElement kd_11[] = { {0, 0, 115.0} };
//  自由度2
SparseElement kd_12[] = { {1, 1, 650.0} };
//  自由度3
SparseElement kd_13[] = { {2, 2, 225.0} };
//  自由度4
SparseElement kd_14[] = { {3, 3, 500.0} };
//  自由度5
SparseElement kd_15[] = { {4, 4, 420.0} };
//  自由度6
SparseElement kd_16[] = { {5, 5, 520.0} };

//  PDゲインの構造体のポインタ
const SparseElement* kp_rows[] = {
  kp_11, kp_12, kp_13, kp_14, kp_15, kp_16
};

const SparseElement* kd_rows[] = {
  kd_11, kd_12, kd_13, kd_14, kd_15, kd_16
};

//(P)  PDゲインの構造体が持つ要素数
int kp_row_sizes[6] = {
  1, 1, 1, 1, 1, 1
};

int kd_row_sizes[6] = {
  1, 1, 1, 1, 1, 1
};

//--制御入力値関連--------------------------------------------------------------------
//  (POT No.[44, 43, 42, 41, 40, 39])
//(P) -制御入力係数（各自由度　最大6要素）--------------------------------------------------------------------
//  自由度1
SparseElement input_coef_11[] = { {0, 0, 40000.0} };
//  自由度2
SparseElement input_coef_12[] = { {1, 1, 20000.0} };
//  自由度3
SparseElement input_coef_13[] = { {2, 2, 30000.0} };
//  自由度4
SparseElement input_coef_14[] = { {3, 3, 200000.0} };
//  自由度5
SparseElement input_coef_15[] = { {4, 4, 30000.0} };
//  自由度6
SparseElement input_coef_16[] = { {5, 5, 30000.0} };

//  制御入力係数の構造体のポインタ
const SparseElement* input_coef_rows[] = {
  input_coef_11, input_coef_12, input_coef_13, input_coef_14, input_coef_15, input_coef_16
};

//(P)  制御入力係数の構造体が持つ要素数
const int input_coef_row_sizes[6] = {
  1, 1, 1, 1, 1, 1
};

//--z3外乱値関連--------------------------------------------------------------------
//  (POT No.[44, 43, 42, 41, 40, 39])
//(P) -z3外乱値係数（各自由度　最大6要素）--------------------------------------------------------------------
/*//  自由度1*/
SparseElement z3_11[] = { {0, 0, 1.0} };
//  自由度2
SparseElement z3_12[] = { {1, 1, 1.0} };
//  自由度3
SparseElement z3_13[] = { {2, 2, 1.0} };
//  自由度4
SparseElement z3_14[] = { {3, 3, 1.0} };
//  自由度5
SparseElement z3_15[] = { {4, 4, 1.0} };
//  自由度6
SparseElement z3_16[] = { {5, 5, 1.0} };

//  z3外乱値係数の構造体のポインタ
const SparseElement* z3_rows[] = {
  z3_11, z3_12, z3_13, z3_14, z3_15, z3_16
};

//(P)  z3外乱値係数の構造体が持つ要素数
int z3_row_sizes[6] = {
  1, 1, 1, 1, 1, 1
};

//--オブザーバーゲイン関連--------------------------------------------------------------------
//  オブザーバーゲイン計算関数用変数
float beta1[6] = {0};
float beta2[6] = {0};
float beta3[6] = {0};

//(P)  オブザーバーの極(-λ₀の重根)(POT No.[44, 43, 42, 41, 40, 39])  
/*No1*/
float lamda_0[6] = {800.0, 300.0, 300.0, 700.0, 300.0, 300.0};

//  オブザーバーゲインの構造体
struct SparseElementBeta {
  int row;
  int col;
  float* value;
};

//(P) -角度推定オブザーバーゲイン（各自由度　最大6要素）--------------------------------------------------------------------
//  自由度1
SparseElementBeta beta11[] = { {0, 0, &beta1[0]} };
//  自由度2
SparseElementBeta beta12[] = { {1, 1, &beta1[1]} };
//  自由度3
SparseElementBeta beta13[] = { {2, 2, &beta1[2]} };
//  自由度4
SparseElementBeta beta14[] = { {3, 3, &beta1[3]} };
//  自由度5
SparseElementBeta beta15[] = { {4, 4, &beta1[4]} };
//  自由度6
SparseElementBeta beta16[] = { {5, 5, &beta1[5]} };

//(P) -角速度推定オブザーバーゲイン（各自由度　最大6要素）--------------------------------------------------------------------
//  自由度1
SparseElementBeta beta21[] = { {0, 0, &beta2[0]} };
//  自由度2
SparseElementBeta beta22[] = { {1, 1, &beta2[1]} };
//  自由度3
SparseElementBeta beta23[] = { {2, 2, &beta2[2]} };
//  自由度4
SparseElementBeta beta24[] = { {3, 3, &beta2[3]} };
//  自由度5
SparseElementBeta beta25[] = { {4, 4, &beta2[4]} };
//  自由度6
SparseElementBeta beta26[] = { {5, 5, &beta2[5]} };

//(P) -外乱推定オブザーバーゲイン（各自由度　最大6要素）--------------------------------------------------------------------
//  自由度1
SparseElementBeta beta31[] = { {0, 0, &beta3[0]} };
//  自由度2
SparseElementBeta beta32[] = { {1, 1, &beta3[1]} };
//  自由度3
SparseElementBeta beta33[] = { {2, 2, &beta3[2]} };
//  自由度4
SparseElementBeta beta34[] = { {3, 3, &beta3[3]} };
//  自由度5
SparseElementBeta beta35[] = { {4, 4, &beta3[4]} };
//  自由度6
SparseElementBeta beta36[] = { {5, 5, &beta3[5]} };

//  オブザーバーゲインの構造体のポインタ
SparseElementBeta* beta_rows[] = {
  beta11, beta12, beta13, beta14, beta15, beta16, //beta1
  beta21, beta22, beta23, beta24, beta25, beta26, //beta2
  beta31, beta32, beta33, beta34, beta35, beta36  //beta3
};

//(P)  オブザーバーゲインの構造体が持つ要素数
int beta_row_sizes[18] = {
  1, 1, 1, 1, 1, 1, //beta1
  1, 1, 1, 1, 1, 1, //beta2
  1, 1, 1, 1, 1, 1  //beta3
};

//---ADRCパラメータの変更（GUIから受信）--------------------------------------------------------------------
//  書き換え用のポインタ（kp_rows などは const なので、同じ配列を指す書き換え可能なポインタを別に持つ）
SparseElement* const kp_rows_param[] = {
  kp_11, kp_12, kp_13, kp_14, kp_15, kp_16
};

SparseElement* const kd_rows_param[] = {
  kd_11, kd_12, kd_13, kd_14, kd_15, kd_16
};

SparseElement* const input_coef_rows_param[] = {
  input_coef_11, input_coef_12, input_coef_13, input_coef_14, input_coef_15, input_coef_16
};

//  受け付けるパラメータの範囲（並び: kp, kd, input_coef, lamda_0）
const float param_min[4] = {KP_MIN, KD_MIN, INPUT_COEF_MIN, LAMDA_0_MIN};
const float param_max[4] = {KP_MAX, KD_MAX, INPUT_COEF_MAX, LAMDA_0_MAX};

//  以下はスレッド間で共有するので adc_lock を取ってから読み書きする
//  検査に通って反映を待っているパラメータ（並びは受信メッセージと同じ）
float param_pending[SUBSCRIBE3];
//  反映待ちのパラメータがあるか（0:なし, 1:あり）
int param_pending_flag = 0;
//  最後に処理した seq と結果（publish用）
float param_ack_seq = 0.0;
float param_result = 0.0;
//  現在のパラメータと .ino の初期値（publish用。[4*i+0..3] = 自由度 i の kp, kd, input_coef, lamda_0）
float param_current[24];
float param_default[24];

//  設定中のパラメータをすぐにpublishする要求（受信を処理したとき1）
volatile int param_pub_request = 0;
//  設定中のパラメータをpublishする間隔のカウンタ
int param_pub_count = 0;

//---追加の制御入力u--------------------------------------------------------------------
//  時間変数
float t_start[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//　1つ前の目標値
float Previous_POT_desired[6] = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};

//  目標値の一時保存
float tmp_POT_desired[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  FF入力時間
float FF_time[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  FFパラメータ u = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t
//  a
float FF_param_a[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  b
float FF_param_b[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  c
float FF_param_c[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  d
float FF_param_d[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  e
float FF_param_e[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//  追加の制御入力u
float outputAdd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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
  float POT0;
  float POT1;
  float POT2;
  float POT3;
  float POT4;
  float POT5;
};

//  標本数
#define LPF_KOSUU 18

//  使用するチャンネル数
#define NUM_CHANNELS 6

//  CircularBufferを使用してリングバッファを作成
CircularBuffer<float, LPF_KOSUU> buffers[NUM_CHANNELS]; //  各チャンネル用のリングバッファ

//  移動平均法での合計値と平均値格納
float pot_sum[NUM_CHANNELS] = {0}; //  各チャンネルの値の合計
float POT[NUM_CHANNELS] = {0};      //  各チャンネルの移動平均値

//---スレッド間で共有リソースへのアクセスを制御するための排他制御（mutex: ミューテックス）を定義--------------------------------------------------------------------
Threads::Mutex adc_lock;

//---subscribeの初回判定--------------------------------------------------------------------
int sub_count = 0;

//---角速度計算--------------------------------------------------------------------
//  リングバッファの設定（データサイズは5点、6種類のデータに対応）
CircularBuffer<float, 5> omegaBuffers[6];

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

    //  周期確認用
    /*
    unsigned long now = micros();
    static unsigned long last = 0;

    Serial.println(now - last);*/

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
      //  subscribeしたメッセージを目標値に格納
      for(int i = 0; i < POT_DESIRED; i++){
        if(sub_count == 0){
          sub[i] = POT_desired[i];
        }

        POT_desired[i] = sub[i];
      }

      //  subscribeしたメッセージをFFパラメータに格納
      for(int i = 0; i < POT_DESIRED; i++){
        int idx = i * 6;

        FF_param_a[i] = sub2[idx];
        FF_param_b[i] = sub2[idx+1];
        FF_param_c[i] = sub2[idx+2];
        FF_param_d[i] = sub2[idx+3];
        FF_param_e[i] = sub2[idx+4];
        FF_time[i] = sub2[idx+5];

      }

      //  GUIから受信したADRCパラメータを反映（ESO・ADRCの計算の前に行う）
      if(param_pending_flag == 1){
        applyADRCParam();
        param_pending_flag = 0;
      }
      
      //  publishメッセージの配列にPOT値を格納
      for(int i = 0; i < ANALOG_IN_CH; i++){
        pub[i] = (uint16_t)round(POT_realized[i]);
      }

      /*  publishメッセージの配列に微分値(絶対値)を格納
      for (int i = 0; i < OMEGA; i++){
        pub[i+6] = (uint16_t)round(abs(derivatives[i]));
      }*/

      /*  publishメッセージの配列に目標値を格納*/
      for (int i = 0; i < POT_DESIRED; i++){
        pub[i+6] = (uint16_t)round(POT_desired[i]);
      }

      /*  publishメッセージの配列にVEABを格納*/
      for (int i = 0; i < PUBLISH2; i++){
        pub2[i] = outputADRC[i] + outputAdd[i];
      }

      /*  publishメッセージの配列にz3を格納*/
      for (int i = 0; i < PUBLISH3; i++){
        pub3[i] = z3[i];
      }

      /*  publishメッセージの配列にz1(角度の推定値)を格納（並びはz3と同じ）*/
      for (int i = 0; i < PUBLISH4; i++){
        pub4[i] = z1[i];
      }

      /*  publishメッセージの配列に角速度(derivatives)を格納（並びはz3と同じ）*/
      for (int i = 0; i < PUBLISH5; i++){
        pub5[i] = derivatives[i];
      }

      /*  publishメッセージの配列にz2(角速度の推定値)を格納（並びはz3と同じ）*/
      for (int i = 0; i < PUBLISH6; i++){
        pub6[i] = z2[i];
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
    //SerialPrint_function();

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

    //  周期確認用
    //last = now;

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

    //  Publlishするメッセージmsg_pub2に格納
    for (size_t i = 0; i < PUBLISH2; i++) {
      msg_pub2.data.data[i] = pub2[i];
    }
    //  メッセージをトピックに送信
    RCSOFTCHECK(rcl_publish(&publisher2, &msg_pub2, NULL));

    //  Publlishするメッセージmsg_pub3に格納
    for (size_t i = 0; i < PUBLISH3; i++) {
      msg_pub3.data.data[i] = pub3[i];
    }
    //  メッセージをトピックに送信
    RCSOFTCHECK(rcl_publish(&publisher3, &msg_pub3, NULL));

    //  Publlishするメッセージmsg_pub4(z1)に格納
    for (size_t i = 0; i < PUBLISH4; i++) {
      msg_pub4.data.data[i] = pub4[i];
    }
    //  メッセージをトピックに送信
    RCSOFTCHECK(rcl_publish(&publisher4, &msg_pub4, NULL));

    //  Publlishするメッセージmsg_pub5(derivatives)に格納
    for (size_t i = 0; i < PUBLISH5; i++) {
      msg_pub5.data.data[i] = pub5[i];
    }
    //  メッセージをトピックに送信
    RCSOFTCHECK(rcl_publish(&publisher5, &msg_pub5, NULL));

    //  Publlishするメッセージmsg_pub6(z2)に格納
    for (size_t i = 0; i < PUBLISH6; i++) {
      msg_pub6.data.data[i] = pub6[i];
    }
    //  メッセージをトピックに送信
    RCSOFTCHECK(rcl_publish(&publisher6, &msg_pub6, NULL));

    //  設定中のADRCパラメータ（PARAM_PUB_EVERY回に1回と、パラメータの受信を処理した直後）
    param_pub_count++;
    if (param_pub_count >= PARAM_PUB_EVERY || param_pub_request == 1) {
      param_pub_count = 0;
      param_pub_request = 0;
      {
        //  スレッドセーフ(制御スレッドが書き換え中の値を読まない)
        Threads::Scope scope(adc_lock);
        msg_pub7.data.data[0] = param_ack_seq;
        msg_pub7.data.data[1] = param_result;
        for (size_t i = 0; i < 24; i++) {
          msg_pub7.data.data[2 + i] = param_current[i];
          msg_pub7.data.data[26 + i] = param_default[i];
        }
      }
      //  メッセージをトピックに送信（送信待ちの間に制御スレッドを止めないよう、ロックの外で送る）
      RCSOFTCHECK(rcl_publish(&publisher7, &msg_pub7, NULL));
    }
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

void subscription_callback2(const void * msgin)
{
  //  受け取ったメッセージmsginをstd_msgs__msg__Float32MultiArray型にキャスト
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  //  Subscribeしたメッセージを格納
  for (size_t i = 0; i < SUBSCRIBE2; i++) {
    sub2[i] = msg->data.data[i];
  }
  
}

//  ADRCパラメータのSubscribe関数（ROS2）
//  受信したメッセージを検査し、通ったものだけを制御スレッドへ渡す（反映は制御スレッドの周期の先頭で行う）
void subscription_callback3(const void * msgin)
{
  //  受け取ったメッセージmsginをstd_msgs__msg__Float32MultiArray型にキャスト
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  //  検査（ロックの外で行う）
  float seq = 0.0;
  int result = checkADRCParam(msg, &seq);

  //  スレッドセーフ(制御スレッドが反映中の値を書き換えない)
  Threads::Scope scope(adc_lock);
  if(result == 1){
    for (size_t i = 0; i < SUBSCRIBE3; i++) {
      param_pending[i] = msg->data.data[i];
    }
    param_pending_flag = 1;
  } else{
    //  不正なメッセージは反映せず、理由をpublishで返す
    param_ack_seq = seq;
    param_result = result;
    param_pub_request = 1;
  }

}

//  自由度 i のパラメータ p (0:kp, 1:kd, 2:input_coef) の要素を返す。変更できない場合は NULL
//  （kp, kd は対角要素(col == i)。input_coef はADRC計算式の分母に先頭要素を使うので、先頭が対角要素のときだけ）
SparseElement* findParamElement(int i, int p){
  if(i < 0 || i >= ADRC_PARAM_DOF){
    return NULL;
  }

  if(p == 2){
    if(input_coef_row_sizes[i] > 0 && input_coef_rows_param[i][0].col == i){
      return &input_coef_rows_param[i][0];
    }
    return NULL;
  }

  SparseElement* row = (p == 0) ? kp_rows_param[i] : kd_rows_param[i];
  int size = (p == 0) ? kp_row_sizes[i] : kd_row_sizes[i];
  for(int k = 0; k < size; k++){
    if(row[k].col == i){
      return &row[k];
    }
  }
  return NULL;
}

//  受信したADRCパラメータの検査（戻り値 1:反映してよい, 負:拒否の理由。seq_out には seq を返す（seqが不正なら0））
int checkADRCParam(const std_msgs__msg__Float32MultiArray * msg, float * seq_out){
  *seq_out = 0.0;

  //  要素数
  if(msg->data.size != (size_t)SUBSCRIBE3){
    return -1;
  }

  //  seq（1以上の整数。floatで正確に表せる範囲）
  float seq = msg->data.data[0];
  if(!isfinite(seq) || seq < 1.0 || seq > 16777216.0 || seq != floorf(seq)){
    return -2;
  }
  *seq_out = seq;

  //  パラメータ（NaN は「変更しない」）
  for(int i = 0; i < 6; i++){
    for(int p = 0; p < 4; p++){
      float v = msg->data.data[1 + 4*i + p];
      if(isnan(v)){
        continue;
      }
      if(!isfinite(v)){
        return -3;
      }
      if(i >= ADRC_PARAM_DOF || (p < 3 && findParamElement(i, p) == NULL)){
        return -5;
      }
      if(v < param_min[p] || v > param_max[p]){
        return -4;
      }
    }
  }
  return 1;
}

//  自由度 i のパラメータ p (0:kp, 1:kd, 2:input_coef, 3:lamda_0) の現在値（変更できない自由度は NaN）
float getADRCParam(int i, int p){
  if(i < 0 || i >= ADRC_PARAM_DOF){
    return NAN;
  }
  if(p == 3){
    return lamda_0[i];
  }
  SparseElement* e = findParamElement(i, p);
  return (e == NULL) ? NAN : e->value;
}

//  publish用の現在のパラメータを更新
void updateParamCurrent(){
  for(int i = 0; i < 6; i++){
    for(int p = 0; p < 4; p++){
      param_current[4*i + p] = getADRCParam(i, p);
    }
  }
}

//  自由度 i のESOの状態を初期化（角度の推定値は実測値、それ以外は0）
void resetESO(int i){
  z1[i] = POT_realized[i];
  z2[i] = 0.0;
  z3[i] = 0.0;
  dz1[i] = 0.0;
  dz2[i] = 0.0;
  dz3[i] = 0.0;
  outputADRC[i] = 0.0;
}

//  検査済みのADRCパラメータを反映する（制御スレッドから、adc_lock を取った状態で呼ぶこと）
void applyADRCParam(){
  int lamda_changed = 0;

  for(int i = 0; i < ADRC_PARAM_DOF; i++){
    int changed = 0;

    for(int p = 0; p < 4; p++){
      float v = param_pending[1 + 4*i + p];
      if(isnan(v)){
        continue;
      }
      if(p == 3){
        lamda_0[i] = v;
        lamda_changed = 1;
      } else{
        findParamElement(i, p)->value = v;
      }
      changed = 1;
    }

    //  ESOの状態が NaN/inf の自由度（input_coef=0 で動かしていた自由度など）はパラメータを変えても回復しないので初期化する
    if(changed && !(isfinite(z1[i]) && isfinite(z2[i]) && isfinite(z3[i]) && isfinite(outputADRC[i]))){
      resetESO(i);
    }
  }

  //  lamda_0 が変わったらオブザーバーゲインを再計算
  if(lamda_changed){
    computeBeta();
  }

  updateParamCurrent();
  param_ack_seq = param_pending[0];
  param_result = 1;
  param_pub_request = 1;
}

//  Publish用 Float32MultiArray メッセージ変数のメモリの確保と初期化（msg_pub2 などと同じ手順）
void initPubMsgFloat(std_msgs__msg__Float32MultiArray * m, size_t n){
  m->data.capacity = n; //  data配列の最大要素数=メッセージの要素数
  m->data.size = n; //  メッセージの要素数
  m->data.data = (float*)malloc(n * sizeof(float));  //　data配列に必要なメモリを動的に確保
  m->layout.dim.capacity = 1;  //  1-dimentional array: vector（次元の数）
  m->layout.dim.size = 1;  //  メッセージのデータの次元数
  m->layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(m->layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  m->layout.dim.data[0].label.capacity = 20;
  m->layout.dim.data[0].label.data = (char*) malloc(m->layout.dim.data[0].label.capacity * sizeof(char));
  m->layout.dim.data[0].label.data[0] = '\0';
  m->layout.dim.data[0].label.size = 0;
  m->layout.dim.data[0].size = n; //  publishするメッセージの次元のサイズ
  m->layout.dim.data[0].stride = n; //　publishするメッセージの更新ストライド
  for(size_t i = 0; i < n; i++){
    m->data.data[i] = 0.0;
  }
}

//  Subscribe用 Float32MultiArray メッセージ変数のメモリの確保と初期化（msg_sub2 などと同じ手順）
void initSubMsgFloat(std_msgs__msg__Float32MultiArray * m, size_t capacity){
  m->data.capacity = capacity;  //  data配列の最大要素数
  m->data.size = 0;  //　初期状態でメッセージを受け取ることを保証（初期化）
  m->data.data = (float*)malloc(capacity * sizeof(float));  //　data配列に必要なメモリを動的に確保
  m->layout.dim.capacity = 1;  //  1-dimentional array: vector
  m->layout.dim.size = 0;  //　配列の次元情報を初期化
  m->layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(m->layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  m->layout.dim.data[0].label.capacity = 20;
  m->layout.dim.data[0].label.size = 0;
  m->layout.dim.data[0].label.data = (char*) malloc(m->layout.dim.data[0].label.capacity * sizeof(char));
  for(size_t i = 0; i < capacity; i++){
    m->data.data[i] = 0.0;
  }
}

//  オブザーバーゲイン関数
void computeBeta(){
  for(int i = 0; i < 6; i++){
    float w = lamda_0[i];
    
    beta1[i] = 3 * w;
    beta2[i] = 3 * w * w;
    beta3[i] = w * w * w; 
  }
}

//  ESO関数
void ESO(int index){
  //  1回目のみ実現値を推定値に格納
  if(eso_count == 0){
    for(int i = 0; i < 6; i++){
      z1[i] = POT_realized[i];

      eso_count = 1;
    }
  }

  //  初期化
  float sum_beta1 = 0.0;
  float sum_beta2 = 0.0;
  float sum_beta3 = 0.0;
  float sum_z3 = 0.0;
  float sum_input = 0.0;

  //  推定誤差計算
  for(int k = 0; k < beta_row_sizes[index]; k++){
    sum_beta1 += (*beta_rows[index][k].value) * (POT_realized[beta_rows[index][k].col] - z1[beta_rows[index][k].col]);
    sum_beta2 += (*beta_rows[index+6][k].value) * (POT_realized[beta_rows[index+6][k].col] - z1[beta_rows[index+6][k].col]);
    sum_beta3 += (*beta_rows[index+12][k].value) * (POT_realized[beta_rows[index+12][k].col] - z1[beta_rows[index+12][k].col]);
  }

  //  外乱値の計算
  for(int k = 0; k < z3_row_sizes[index]; k++){
    sum_z3 += (z3_rows[index][k].value) * z3[z3_rows[index][k].col];
  }

  //  制御入力の計算
  for(int k = 0; k < input_coef_row_sizes[index]; k++){
    sum_input += (input_coef_rows[index][k].value) * outputADRC[input_coef_rows[index][k].col];
  }

  //  角度の推定
  dz1[index] = z2[index] + sum_beta1;
  //  角速度の推定
  dz2[index] = sum_z3 + sum_beta2 + sum_input;
  //  外乱（動特性）の推定
  dz3[index] = sum_beta3;

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
  for(int i = 0; i < 6; i++){
    errors[i] = POT_desired[i] - z1[i];
  }

  //  初期化
  float sum_kp = 0.0;
  float sum_kd = 0.0;

  //  PD制御値計算
  for(int k = 0; k < kp_row_sizes[index]; k++){
    sum_kp += (kp_rows[index][k].value) * errors[kp_rows[index][k].col];
  }

  for(int k = 0; k < kd_row_sizes[index]; k++){
    sum_kd += (kd_rows[index][k].value) * derivatives[kd_rows[index][k].col];
  }

  //  ADRC計算
  outputADRC[index] = (-z3[index] + sum_kp - sum_kd) / input_coef_rows[index][0].value;

  //  目標値の確認
  POT_desired_change(index);

  //  追加の制御入力u計算
  AddInput(index);

  //  アンドロイドのアクチュエータの要件（ポテンショメータの正方向と入力の正方向を一致させる）
  outputADRC_direct[index] = (outputADRC[index] + outputAdd[index]) * direction[index];

  //  VEAB1とVEAB2に与えるPWMの値を計算し格納
  Result veab = calculate_veab_Values(outputADRC_direct[index], index);
  ADRC_PWM[2*index] = veab.veab_value1;   //0, 2, 4, 6, 8, 10ピンへ
  ADRC_PWM[2*index+1] = veab.veab_value2; //1, 3, 5, 7, 9, 11ピンへ

}

//(P)  VEAB1とVEAB2に与えるPWMの値の計算関数(アクチュエータの両ポートの圧力が釣り合うPWN値を基準に足し引きを行う)
Result calculate_veab_Values(float outputADRC_direct, int i) {
  Result result;
  if(i == 0){
    result.veab_value1 = 128 + (outputADRC_direct / 2.0);  
    result.veab_value2 = 128 - (outputADRC_direct / 2.0);
  } else if(i == 1){
    result.veab_value1 = 128 + (outputADRC_direct / 2.0);  
    result.veab_value2 = 128 - (outputADRC_direct / 2.0);
  } else if(i == 2){
    result.veab_value1 = 128 + (outputADRC_direct / 2.0);  
    result.veab_value2 = 128 - (outputADRC_direct / 2.0);
  } else if(i == 3){
    result.veab_value1 = 128 + (outputADRC_direct / 2.0);  
    result.veab_value2 = 128 - (outputADRC_direct / 2.0);
  } else if(i == 4){
    result.veab_value1 = 128 + (outputADRC_direct / 2.0);  
    result.veab_value2 = 128 - (outputADRC_direct / 2.0);
  } else{
    result.veab_value1 = 128 + (outputADRC_direct / 2.0);  
    result.veab_value2 = 128 - (outputADRC_direct / 2.0);
  }

  result.veab_value1 = max(0, min(255, int(result.veab_value1)));
  result.veab_value2 = max(0, min(255, int(result.veab_value2)));

  return result;
}

//  追加の制御入力u関数（DMPのようなもの）
void AddInput(int index) {
  float t = millis();
  //float r2 = POT_desired[index];
  //float r1 = Previous_POT_desired[index];
  //float dr = fabs(r2 - r1);

  /*修正前2026/06/26 20:23
  float s = ((t - t_start[index]) / 1000.0) / FF_time[index];
  s = max(0, min(1.0, s));

  // --- FF制御入力 ---
  outputAdd[index] = FF_param_a[index] * s * s * s * s * s + FF_param_b[index] * s * s * s * s + FF_param_c[index] * s * s * s + FF_param_d[index] * s * s + FF_param_e[index] *s;

  if(s == 1){
    outputAdd[index] = 0;
  }
  */

  /*修正後2026/06/26 20:30*/
  float timeFF = (t - t_start[index]) / 1000.0;
  timeFF = max(0, min(FF_time[index], timeFF));

  // --- FF制御入力 ---
  outputAdd[index] = FF_param_a[index] * timeFF * timeFF * timeFF * timeFF * timeFF + FF_param_b[index] * timeFF * timeFF * timeFF * timeFF + FF_param_c[index] * timeFF * timeFF * timeFF + FF_param_d[index] * timeFF * timeFF + FF_param_e[index] *timeFF;

  //  FF時間終了
  if(timeFF == FF_time[index]){
    //  入力0
    outputAdd[index] = 0;

    //  FFパラメータを初期化
    FF_param_a[index] = 0.0;
    FF_param_b[index] = 0.0;
    FF_param_c[index] = 0.0;
    FF_param_d[index] = 0.0;
    FF_param_e[index] = 0.0;
    FF_time[index] = 0.0;
  }

}

//  目標値保存関数
void POT_desired_change(int index) {

  //　初回の目標値を保存
  if(Previous_POT_desired[index] == -1.0) {
    Previous_POT_desired[index] = POT_desired[index];
    tmp_POT_desired[index] = POT_desired[index];
  }

  //  1つ前の目標値の更新
  if(POT_desired[index] != tmp_POT_desired[index]){
    Previous_POT_desired[index] = tmp_POT_desired[index];
    
    //  目標値受信時間
    t_start[index] = millis();
  }

  //  一時目標値関数に保存
  tmp_POT_desired[index] = POT_desired[index];

}

//  微分値計算関数
float calculateDerivative(CircularBuffer<float, 5> &buffer) {
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
  /*
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
  Serial.print(FF_param_a[0]);
  Serial.print(",");
  Serial.print(FF_param_b[0]);
  Serial.print(",");
  Serial.print(FF_param_c[0]);
  Serial.print(",");
  Serial.print(FF_param_d[0]);
  Serial.print(",");
  Serial.print(FF_param_e[0]);
  Serial.print(",");
  Serial.print(FF_time[0]);
  Serial.print(",");
  Serial.print(outputADRC[0]);
  Serial.print(",");
  Serial.print(outputAdd[0]);
  Serial.print(",");
  Serial.print(outputADRC_direct[0]* direction[0]);
  Serial.print(",");
  Serial.print(POT_desired[0]);*/
  /*
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
  Serial.print(FF_param_a[1]);
  Serial.print(",");
  Serial.print(FF_param_b[1]);
  Serial.print(",");
  Serial.print(FF_param_c[1]);
  Serial.print(",");
  Serial.print(FF_param_d[1]);
  Serial.print(",");
  Serial.print(FF_param_e[1]);
  Serial.print(",");
  Serial.print(FF_time[1]);
  Serial.print(",");
  Serial.print(outputADRC[1]);
  Serial.print(",");
  Serial.print(outputAdd[1]);
  Serial.print(",");
  Serial.print(outputADRC_direct[1]* direction[1]);
  Serial.print(",");
  Serial.print(POT_desired[1]);*/
  /*
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
  Serial.print(FF_param_a[2]);
  Serial.print(",");
  Serial.print(FF_param_b[2]);
  Serial.print(",");
  Serial.print(FF_param_c[2]);
  Serial.print(",");
  Serial.print(FF_param_d[2]);
  Serial.print(",");
  Serial.print(FF_param_e[2]);
  Serial.print(",");
  Serial.print(FF_time[2]);
  Serial.print(",");
  Serial.print(outputADRC[2]);
  Serial.print(",");
  Serial.print(outputAdd[2]);
  Serial.print(",");
  Serial.print(outputADRC_direct[2]* direction[2]);
  Serial.print(",");
  Serial.print(POT_desired[2]);*/
  /*
  Serial.print(",");*/
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
  Serial.print(FF_param_a[3]);
  Serial.print(",");
  Serial.print(FF_param_b[3]);
  Serial.print(",");
  Serial.print(FF_param_c[3]);
  Serial.print(",");
  Serial.print(FF_param_d[3]);
  Serial.print(",");
  Serial.print(FF_param_e[3]);
  Serial.print(",");
  Serial.print(FF_time[3]);
  Serial.print(",");
  Serial.print(outputADRC[3]);
  Serial.print(",");
  Serial.print(outputAdd[3]);
  Serial.print(",");
  Serial.print(outputADRC_direct[3]* direction[3]);
  Serial.print(",");
  Serial.println(POT_desired[3]);
  /*
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
  Serial.print(FF_param_a[4]);
  Serial.print(",");
  Serial.print(FF_param_b[4]);
  Serial.print(",");
  Serial.print(FF_param_c[4]);
  Serial.print(",");
  Serial.print(FF_param_d[4]);
  Serial.print(",");
  Serial.print(FF_param_e[4]);
  Serial.print(",");
  Serial.print(FF_time[4]);
  Serial.print(",");
  Serial.print(outputADRC[4]);
  Serial.print(",");
  Serial.print(outputAdd[4]);
  Serial.print(",");
  Serial.print(outputADRC_direct[4]* direction[4]);
  Serial.print(",");
  Serial.print(POT_desired[4]);*/
  /*
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
  Serial.print(FF_param_a[5]);
  Serial.print(",");
  Serial.print(FF_param_b[5]);
  Serial.print(",");
  Serial.print(FF_param_c[5]);
  Serial.print(",");
  Serial.print(FF_param_d[5]);
  Serial.print(",");
  Serial.print(FF_param_e[5]);
  Serial.print(",");
  Serial.print(FF_time[5]);
  Serial.print(",");
  Serial.print(outputADRC[5]);
  Serial.print(",");
  Serial.print(outputAdd[5]);
  Serial.print(",");
  Serial.print(outputADRC_direct[5]* direction[5]);
  Serial.print(",");
  Serial.println(POT_desired[5]);*/

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
  set_microros_native_ethernet_udp_transports(teensy_mac, teensy_ip, agent_ip, 9991);

  //  シリアル通信を初期化する。ボーレートは9600bps（デバック時に用いる。シリアルモニタを開いてから書き込みを行う）
  //Serial.begin(9600);

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

  //  メッセージ変数msg_pub2に対して、メモリの確保と初期化
  msg_pub2.data.capacity = PUBLISH2; //  data配列の最大要素数=メッセージの要素数
  msg_pub2.data.size = 0;  //　メッセージ送信時のデータの初期化の保証
  msg_pub2.data.data = (float*)malloc(msg_pub2.data.capacity * sizeof(float));  //　data配列に必要なメモリを動的に確保
  msg_pub2.layout.dim.capacity = 1;  //  1-dimentional array: vector（次元の数）
  msg_pub2.layout.dim.size = 0;  //  配列の次元情報を初期化
  msg_pub2.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_pub2.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension)); //  MultiArrayDimension のためのメモリを確保
  for(size_t i = 0; i < msg_pub2.layout.dim.capacity; i++){  //  ラベルフィールドの設定(コピペ)
    msg_pub2.layout.dim.data[i].label.capacity = 20;
    msg_pub2.layout.dim.data[i].label.size = 0;
    msg_pub2.layout.dim.data[i].label.data = (char*) malloc(msg_pub2.layout.dim.data[i].label.capacity * sizeof(char));
  }

  //  メッセージ変数msg_pub3に対して、メモリの確保と初期化
  msg_pub3.data.capacity = PUBLISH3; //  data配列の最大要素数=メッセージの要素数
  msg_pub3.data.size = 0;  //　メッセージ送信時のデータの初期化の保証
  msg_pub3.data.data = (float*)malloc(msg_pub3.data.capacity * sizeof(float));  //　data配列に必要なメモリを動的に確保
  msg_pub3.layout.dim.capacity = 1;  //  1-dimentional array: vector（次元の数）
  msg_pub3.layout.dim.size = 0;  //  配列の次元情報を初期化
  msg_pub3.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_pub3.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension)); //  MultiArrayDimension のためのメモリを確保
  for(size_t i = 0; i < msg_pub3.layout.dim.capacity; i++){  //  ラベルフィールドの設定(コピペ)
    msg_pub3.layout.dim.data[i].label.capacity = 20;
    msg_pub3.layout.dim.data[i].label.size = 0;
    msg_pub3.layout.dim.data[i].label.data = (char*) malloc(msg_pub3.layout.dim.data[i].label.capacity * sizeof(char));
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

  //  メッセージ変数msg_sub2に対して、メモリの確保と初期化
  msg_sub2.data.capacity = SUBSCRIBE2;  //  data配列の最大要素数=メッセージの要素数
  msg_sub2.data.size = 0;  //　初期状態でメッセージを受け取ることを保証（初期化）
  msg_sub2.data.data = (float*)malloc(msg_sub2.data.capacity * sizeof(float));  //　data配列に必要なメモリを動的に確保
  msg_sub2.layout.dim.capacity = 1;  //  1-dimentional array: vector
  msg_sub2.layout.dim.size = 0;  //　配列の次元情報を初期化
  msg_sub2.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_sub2.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension)); //  MultiArrayDimension のためのメモリを確保
  for(size_t i = 0; i < msg_sub2.layout.dim.capacity; i++){ //  ラベルフィールドの設定(コピペ)
    msg_sub2.layout.dim.data[i].label.capacity = 20;
    msg_sub2.layout.dim.data[i].label.size = 0;
    msg_sub2.layout.dim.data[i].label.data = (char*) malloc(msg_sub2.layout.dim.data[i].label.capacity * sizeof(char));
  }

  //------initialize message variables--------------------------------------------------------------------
  //  2.メッセージ変数の設定段階（送信可能な状態に設定）
  // msg_pub
  msg_pub.data.size = PUBLISH; //  メッセージの要素数を設定（データの実際の使用サイズ）
  msg_pub.layout.dim.size = 1;  //  メッセージのデータの次元数を設定(ex, 1 = 1次元の配列(ベクトル))
  msg_pub.layout.dim.data[0].label.size = strlen(msg_pub.layout.dim.data[0].label.data);  //labelの長さを設定
  msg_pub.layout.dim.data[0].size = PUBLISH; //  publishするメッセージの次元のサイズを設定
  msg_pub.layout.dim.data[0].stride = PUBLISH; //　publishするメッセージの更新ストライドを設定
  //msg_pub2
  msg_pub2.data.size = PUBLISH2; //  メッセージの要素数を設定（データの実際の使用サイズ）
  msg_pub2.layout.dim.size = 1;  //  メッセージのデータの次元数を設定(ex, 1 = 1次元の配列(ベクトル))
  msg_pub2.layout.dim.data[0].label.size = strlen(msg_pub2.layout.dim.data[0].label.data);  //labelの長さを設定
  msg_pub2.layout.dim.data[0].size = PUBLISH2; //  publishするメッセージの次元のサイズを設定
  msg_pub2.layout.dim.data[0].stride = PUBLISH2; //　publishするメッセージの更新ストライドを設定
  //msg_pub3
  msg_pub3.data.size = PUBLISH3; //  メッセージの要素数を設定（データの実際の使用サイズ）
  msg_pub3.layout.dim.size = 1;  //  メッセージのデータの次元数を設定(ex, 1 = 1次元の配列(ベクトル))
  msg_pub3.layout.dim.data[0].label.size = strlen(msg_pub3.layout.dim.data[0].label.data);  //labelの長さを設定
  msg_pub3.layout.dim.data[0].size = PUBLISH3; //  publishするメッセージの次元のサイズを設定
  msg_pub3.layout.dim.data[0].stride = PUBLISH3; //　publishするメッセージの更新ストライドを設定
  //  メッセージ変数を初期化
  for (size_t i = 0; i < PUBLISH; i++) {
    pub[i] = 0; //  pub配列（publishメッセージの値）の初期化
    msg_pub.data.data[i] = pub[i];  //  メッセージのデータ部分に初期化された値を格納
  }
  for (size_t i = 0; i < PUBLISH2; i++) {
    pub2[i] = 0; //  pub配列（publishメッセージの値）の初期化
    msg_pub2.data.data[i] = pub2[i];  //  メッセージのデータ部分に初期化された値を格納
  }
  for (size_t i = 0; i < PUBLISH3; i++) {
    pub3[i] = 0; //  pub配列（publishメッセージの値）の初期化
    msg_pub3.data.data[i] = pub3[i];  //  メッセージのデータ部分に初期化された値を格納
  }
  for (size_t i = 0; i < SUBSCRIBE; i++) {
    msg_sub.data.data[i] = 255.0; //  sub配列（subscribeメッセージの値）の初期化
    sub[i] = msg_sub.data.data[i]; //  メッセージデータをsub配列にコピー
    
  }
  for (size_t i = 0; i < SUBSCRIBE2; i++) {
    msg_sub2.data.data[i] = 0.0; //  sub配列（subscribeメッセージの値）の初期化
    if(i == 5 || i == 11 || i == 17 || i == 23 || i == 29 || i == 35){
      msg_sub2.data.data[i] = 1.0;
    }
    sub2[i] = msg_sub2.data.data[i]; //  メッセージデータをsub配列にコピー
    
  }

  //  追加したメッセージ変数（z1, derivatives, z2, ADRCパラメータ）のメモリの確保と初期化
  initPubMsgFloat(&msg_pub4, PUBLISH4);
  initPubMsgFloat(&msg_pub5, PUBLISH5);
  initPubMsgFloat(&msg_pub6, PUBLISH6);
  initPubMsgFloat(&msg_pub7, PUBLISH7);
  initSubMsgFloat(&msg_sub3, SUBSCRIBE3_CAPACITY);
  for (size_t i = 0; i < PUBLISH4; i++) {
    pub4[i] = 0;
    pub5[i] = 0;
    pub6[i] = 0;
  }

  delay(2000);

  //****************************************
  // micro-ROS functions setup
  //****************************************

  //  ①create default allocator（メモリ管理のためのアロケータ（メモリ確保用の管理機構）を取得）
  allocator = rcl_get_default_allocator();

  //  ②create init_options（サポートオプションを設定。引数もコピペでよい）
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  //  ③create node（引数：(初期化するノード, ノードの名前, ノード空間の名前, サポート構造体)
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //------create subscriber（subscriberの初期化）--------------------------------------------------------------------
  RCCHECK(rclc_subscription_init_default(
    &subscriber,  //  subscriberの構造体を指定
    &node,  //  subscriberが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  subscriberが受け取るメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    SUB_TOPICNAME));  //  subscribeするトピックの名前

  RCCHECK(rclc_subscription_init_default(
    &subscriber2,  //  subscriberの構造体を指定
    &node,  //  subscriberが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  subscriberが受け取るメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    SUB_TOPICNAME2));  //  subscribeするトピックの名前

  RCCHECK(rclc_subscription_init_default(
    &subscriber3,  //  subscriberの構造体を指定
    &node,  //  subscriberが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  subscriberが受け取るメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    SUB_TOPICNAME3));  //  subscribeするトピックの名前

  //------create publisher--------------------------------------------------------------------
  RCCHECK(rclc_publisher_init_default(
    &publisher, //  publisherの構造体を指定
    &node,  //  publisherが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/UInt16MultiArray 型)
    PUB_TOPICNAME));  //  publishするトピックの名前
  
  RCCHECK(rclc_publisher_init_default(
    &publisher2, //  publisher2の構造体を指定
    &node,  //  publisher2が関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    PUB_TOPICNAME2));  //  publishするトピックの名前
  
  RCCHECK(rclc_publisher_init_default(
    &publisher3, //  publisher3の構造体を指定
    &node,  //  publisher3が関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    PUB_TOPICNAME3));  //  publishするトピックの名前

  RCCHECK(rclc_publisher_init_default(
    &publisher4, //  publisher4の構造体を指定
    &node,  //  publisher4が関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    PUB_TOPICNAME4));  //  publishするトピックの名前

  RCCHECK(rclc_publisher_init_default(
    &publisher5, //  publisher5の構造体を指定
    &node,  //  publisher5が関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    PUB_TOPICNAME5));  //  publishするトピックの名前

  RCCHECK(rclc_publisher_init_default(
    &publisher6, //  publisher6の構造体を指定
    &node,  //  publisher6が関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    PUB_TOPICNAME6));  //  publishするトピックの名前

  RCCHECK(rclc_publisher_init_default(
    &publisher7, //  publisher7の構造体を指定
    &node,  //  publisher7が関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/Float32MultiArray 型)
    PUB_TOPICNAME7));  //  publishするトピックの名前

  //------create timer（timerを利用して定期的に実行するtimer_callback関数を設定）--------------------------------------------------------------------
  const unsigned int timer_timeout = PUB_PERIOD_MS;
  RCCHECK(rclc_timer_init_default(
    &timer, //  タイマー構造体
    &support, //  サポート構造体
    RCL_MS_TO_NS(timer_timeout),  //  周期を指定
    timer_callback)); //  コールバック関数の指定

  //------create executor--------------------------------------------------------------------
  //  Executor（実行管理エンジン）を初期化（引数：(初期化するExecutor構造体, Executorが利用するROS2のコンテキスト, Executorが管理するハンドル（ex. subscribe,timer）の数, メモリ割り当て用のカスタムアロケータ)）
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  //  SubscriberをExecutorに追加（引数：(初期化済みのExecutor, 追加するsubscriber構造体, subscriberが受信したメッセージ変数, subscriber用コールバック関数, 新しいデータが届いたときのみコールバックを実行する動作モード)）
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg_sub2, &subscription_callback2, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber3, &msg_sub3, &subscription_callback3, ON_NEW_DATA));
  //  timerをExecutorに追加（引数：(初期化済みのExecutor, 追加するtimer構造体)）
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //======setup関数内での実行処理==============================
  //  VEABの初期化
  /*ピン0,1*/
  analogWrite(aout_channels[0], 128);
  analogWrite(aout_channels[1], 128);
  /*ピン2,3*/
  analogWrite(aout_channels[2], 128);
  analogWrite(aout_channels[3], 128);
  /*ピン4,5*/
  analogWrite(aout_channels[4], 128);
  analogWrite(aout_channels[5], 128);
  /*ピン6,7*/
  analogWrite(aout_channels[6], 128);
  analogWrite(aout_channels[7], 128);
  /*ピン8,9*/
  analogWrite(aout_channels[8], 128);
  analogWrite(aout_channels[9], 128);
  /*ピン28,29*/
  analogWrite(aout_channels[10], 128);
  analogWrite(aout_channels[11], 128);

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

  //  オブザーバーゲイン計算
  computeBeta();

  //  ADRCパラメータの .ino の初期値を保存し、publish用の現在値を作る
  updateParamCurrent();
  for (int i = 0; i < 24; i++){
    param_default[i] = param_current[i];
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
