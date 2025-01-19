//=============================================================================================================================================
//------周期--------------------------------------------------------------------
//　制御周期(単位：ms)
#define CONTROL_PERIOD_MS 1

//　ROSのPublishする周期(単位：ms)
#define PUB_PERIOD_MS 10

//------Topic names--------------------------------------------------------------------
#define SUB_TOPICNAME "/board1/aout"
#define PUB_TOPICNAME "/board1/ain"

//------LEDの定義ピン--------------------------------------------------------------------
#define LED 13

//------AD/DA conv channels--------------------------------------------------------------------
//　アナログ入力のピン数
//#define ANALOG_IN_CH 18
#define ANALOG_IN_CH 6

//　アナログ出力のピン数
#define ANALOG_OUT_CH 12

//  VEABのアナログ出力(14～25),POTのアナログ出力(26～41)
//const int ain_channels[ANALOG_IN_CH] = {14,15,16,17,18,19,20,21,22,23,24,25,26,27,38,39,40,41};
const int ain_channels[ANALOG_IN_CH] = {26,27,38,39,40,41};

// PWMピンの定義
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

//　使用するメッセージの型のライブラリ
#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

//------マクロの定義--------------------------------------------------------------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// ROS2の関数fnが成功したかどうかを確認。成功したら何もせず次の処理に進み、失敗したらerror_loop()を呼び出しエラー状態にする

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// ROS2の関数fnが成功したかどうかを確認。成功したら何もせず次の処理に進み、失敗しても何もせずスルーする

//------オブジェクトの定義--------------------------------------------------------------------
//  使用するROS2メッセージの型
std_msgs__msg__UInt16MultiArray msg_pub;
std_msgs__msg__UInt16MultiArray msg_sub;

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
//　アナログ出力の配列
volatile uint16_t aout[12];

//　アナログ入力の配列
volatile uint16_t ain[6];

//  POT値
volatile uint16_t POT_realized[6] = {0, 0, 0, 0, 0, 0};

//---PID制御--------------------------------------------------------------------
//  PIDゲイン
const float kp[6] = {
  0.7, 2.1, 0.85, 0.2, 1.6, 0.6
};
const float ki[6] = {
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};
const float kd[6] = {
  20.0, 25.0, 20.0, 2.0, 0.0, 0.0
};

//  目標値{腕の閉190-394開, 腕の下287-534上, 上腕の旋回内87-500外, 肘の伸124-635曲, 前腕の旋回内98-900外, 小指側縮48-822伸}
volatile uint16_t POT_desired[6] = {
  390, 548, 113, 220, 130, 500
};

//  各自由度ごとの圧力の正方向とポテンショメータの正方向の対応を整理
const int direction[6] = {-1, -1, 1, -1, -1, -1};

//  各要素(自由度)の誤差
int errors[6] = {0, 0, 0, 0, 0, 0};

//  各要素(自由度)の1ステップ前の誤差
int previous_errors[6] = {0, 0, 0, 0, 0, 0};

//  各要素(自由度)の誤差の積分値
int integral[6] = {0, 0, 0, 0, 0, 0};

//  各要素(自由度)の誤差の微分値
int de[6] = {0, 0, 0, 0, 0, 0};

//  PID制御計算値
float outputPID[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//---VEABへのPWM信号の出力--------------------------------------------------------------------
//  VEABへのPWM出力値用構造体
struct Result {
  int veab_value1;
  int veab_value2;
};

//  VEABへのPWM出力値
int VEAB_desired[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//------ローパスフィルタ--------------------------------------------------------------------
//---RCフィルタ--------------------------------------------------------------------
//  ローパスフィルタの係数　係数a=1/(2*pi*fc*dt + 1)   fc[Hz]:カットオフ周波数、dt[s]:サンプリング周期
//  カットオフ周波数:Hz, サンプリング周期:で設定
const float coef_lpf_veab = 0.75; //VEAB
const float coef_lpf_pot = 0.3;   //POT

//  ローパスフィルタの値保持変数
int veab_filter[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //VEAB

//  ローパスフィルタ用前回の値保持変数
int previous_value_veab[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //VEAB


//  ローパスフィルタ用初回判定
int initial_lpf_veab[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //VEAB

//---移動平均法--------------------------------------------------------------------
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
#define LPF_kosuu 50

//  使用するチャンネル数
#define NUM_CHANNELS 6

//  CircularBufferを使用してリングバッファを作成
CircularBuffer<int, LPF_kosuu> buffers[NUM_CHANNELS]; // 各チャンネル用のリングバッファ

//移動平均法での合計値と平均値格納
long pot_sum[NUM_CHANNELS] = {0}; // 各チャンネルの値の合計
int POT[NUM_CHANNELS] = {0};     // 各チャンネルの移動平均値

//------スレッド間で共有リソースへのアクセスを制御するための排他制御（mutex: ミューテックス）を定義--------------------------------------------------------------------
Threads::Mutex adc_lock;

//=============================================================================================================================================

//------関数の定義--------------------------------------------------------------------
//　エラー関数
void error_loop(){
  while(1){
    //　エラー状態のときLED点滅
    digitalWrite(LED, !digitalRead(LED));
    delay(100);
  }
}

//　スレッドに追加した関数
void thread_callback() {
  while(1) {
    int t0 = millis();

    //====================================
    // write codes from here
    //====================================
    
    if(1) {
      //  スレッドセーフ(アクセスを阻止)
      Threads::Scope scope(adc_lock);

      //---PID制御--------------------------------------------------------------------
      //  移動平均法ローパスフィルタを適用したPOT値をPOT_realizedに格納
      Result_LPF pot = Moving_LPF();
      POT_realized[0] = pot.POT0;
      POT_realized[1] = pot.POT1;
      POT_realized[2] = pot.POT2;
      POT_realized[3] = pot.POT3;
      POT_realized[4] = pot.POT4;
      POT_realized[5] = pot.POT5;

      //  ROS用の配列にPOT値を格納
      for(int i = 0; i < ANALOG_IN_CH; i++){
        ain[i] = POT_realized[i];
      }

    }

    //  PID制御
    PID();

    //  RCローパスフィルタ適用(VEAB)
    for(int i = 0; i < 12; i++){

      //ローパスフィルタ関数呼び出し
      veab_filter[i] = RC_LPF(VEAB_desired[i], previous_value_veab[i], initial_lpf_veab[i], coef_lpf_veab);

      initial_lpf_veab[i] += 1;

      //PWM値に格納
      VEAB_desired[i] = veab_filter[i];

      //前回のVEAB値に格納
      previous_value_veab[i] = veab_filter[i];
      
    }

    //------VEABへ出力--------------------------------------------------------------------
    /*ピン0,1
    analogWrite(aout_channels[0], VEAB_desired[0]);
    analogWrite(aout_channels[1], VEAB_desired[1]);*/
    /*ピン2,3
    analogWrite(aout_channels[2], VEAB_desired[2]);
    analogWrite(aout_channels[3], VEAB_desired[3]);*/
    /*ピン4,5
    analogWrite(aout_channels[4], VEAB_desired[4]);
    analogWrite(aout_channels[5], VEAB_desired[5]);*/
    /*ピン6,7
    analogWrite(aout_channels[6], VEAB_desired[6]);
    analogWrite(aout_channels[7], VEAB_desired[7]);*/
    /*ピン8,9
    analogWrite(aout_channels[8], VEAB_desired[8]);
    analogWrite(aout_channels[9], VEAB_desired[9]);*/
    /*ピン28,29
    analogWrite(aout_channels[10], VEAB_desired[10]);
    analogWrite(aout_channels[11], VEAB_desired[11]);*/

    //====================================
    // to here
    //====================================
    int t1 = millis();
    //Serial.println(t1-t0);

    //　制御周期を満たすように調整
    if( t1-t0 <= CONTROL_PERIOD_MS ){
      // spare cpu slices to other threads for the remaining time
      threads.delay(CONTROL_PERIOD_MS - (t1-t0));
    } else {
      // the designated period is violated
      error_loop();
    }
  }
}

//　Publish関数（ROS2）
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  //　引数last_call_timeは使わない
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    //　Publlishするメッセージmsg_pubに格納
    for (size_t i = 0; i < ANALOG_IN_CH; i++) {
      msg_pub.data.data[i] = ain[i];
    }
    //　メッセージをトピックに送信
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
  }
}

//　Subscribe関数（ROS2）
void subscription_callback(const void * msgin)
{
  //　受け取ったメッセージmsginをstd_msgs__msg__UInt16MultiArray型にキャスト
  const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *)msgin;
  //  Subscribeしたメッセージを格納
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    aout[i] = msg->data.data[i];
  }
  
}

void PID(){
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
}

//VEAB1とVEAB2に与えるPWMの値の計算関数(Ballistic Modeにおける両ポートの値を基準に足し引きを行う)
Result calculate_veab_Values(float outputPID, int i) {
  Result result;
  if(i == 0){
    result.veab_value1 = 140 + (outputPID / 2.0);  
    result.veab_value2 = 116 - (outputPID / 2.0);
  } else if(i == 1){
    result.veab_value1 = 128 + (outputPID / 2.0);  
    result.veab_value2 = 128 - (outputPID / 2.0);
  } else if(i == 2){
    result.veab_value1 = 127 + (outputPID / 2.0);  
    result.veab_value2 = 129 - (outputPID / 2.0);
  } else if(i == 3){
    result.veab_value1 = 136 + (outputPID / 2.0);  
    result.veab_value2 = 120 - (outputPID / 2.0);
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

//  ローパスフィルタ(RCフィルタ)関数
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

//  ローパスフィルタ(移動平均法)関数
Result_LPF Moving_LPF() {
  Result_LPF lpf;

  // 各チャンネルの処理
  for (int i = 0; i < NUM_CHANNELS; i++) {
    int new_value = analogRead(ain_channels[i]); // 新しいアナログ値を読み取る

    // リングバッファが満杯の場合、古い値を取り出して合計から引く
    if (buffers[i].isFull()) {
      pot_sum[i] -= buffers[i].first(); // 最古の値を引く
    }

    // 新しい値をバッファに追加し、合計を更新
    buffers[i].push(new_value);
    pot_sum[i] += new_value;

    // 移動平均を計算
    POT[i] = pot_sum[i] / buffers[i].size(); // バッファ内の現在の要素数で割る
  }

  lpf.POT0 = POT[0];
  lpf.POT1 = POT[1];
  lpf.POT2 = POT[2];
  lpf.POT3 = POT[3];
  lpf.POT4 = POT[4];
  lpf.POT5 = POT[5];

  return lpf;
}

//=============================================================================================================================================

void setup() {
  //　micro-rosの通信手段を設定
  set_microros_transports();
  
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
  //　メッセージ変数msg_pubに対して、メモリの確保と初期化
  msg_pub.data.capacity = ANALOG_IN_CH; //  data配列の最大要素数=メッセージの要素数
  msg_pub.data.size = 0;  //　メッセージ送信時のデータの初期化の保証
  msg_pub.data.data = (uint16_t*)malloc(msg_pub.data.capacity * sizeof(uint16_t));  //　data配列に必要なメモリを動的に確保
  msg_pub.layout.dim.capacity = 1; // 1-dimentional array: vector（次元の数）
  msg_pub.layout.dim.size = 0;  //　配列の次元情報を初期化
  msg_pub.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_pub.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension)); //  MultiArrayDimension のためのメモリを確保
  for(size_t i = 0; i < msg_pub.layout.dim.capacity; i++){  //  ラベルフィールドの設定(コピペ)
    msg_pub.layout.dim.data[i].label.capacity = 20;
    msg_pub.layout.dim.data[i].label.size = 0;
    msg_pub.layout.dim.data[i].label.data = (char*) malloc(msg_pub.layout.dim.data[i].label.capacity * sizeof(char));
  }
  
  //　メッセージ変数msg_subに対して、メモリの確保と初期化
  msg_sub.data.capacity = ANALOG_OUT_CH;  //  data配列の最大要素数=メッセージの要素数
  msg_sub.data.size = 0;  //　初期状態でメッセージを受け取ることを保証（初期化）
  msg_sub.data.data = (uint16_t*)malloc(msg_sub.data.capacity * sizeof(uint16_t));  //　data配列に必要なメモリを動的に確保
  msg_sub.layout.dim.capacity = 1; // 1-dimentional array: vector
  msg_sub.layout.dim.size = 0;  //　配列の次元情報を初期化
  msg_sub.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg_sub.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension)); //  MultiArrayDimension のためのメモリを確保
  for(size_t i = 0; i < msg_sub.layout.dim.capacity; i++){ //  ラベルフィールドの設定(コピペ)
    msg_sub.layout.dim.data[i].label.capacity = 20;
    msg_sub.layout.dim.data[i].label.size = 0;
    msg_sub.layout.dim.data[i].label.data = (char*) malloc(msg_sub.layout.dim.data[i].label.capacity * sizeof(char));
  }

  //------initialize message variables--------------------------------------------------------------------
  //  2.メッセージ変数の設定段階（送信可能な状態に設定）
  msg_pub.data.size = ANALOG_IN_CH; //  メッセージの要素数を設定（データの実際の使用サイズ）
  msg_pub.layout.dim.size = 1;  //  メッセージのデータの次元数を設定(ex, 1 = 1次元の配列(ベクトル))
  msg_pub.layout.dim.data[0].label.size = strlen(msg_pub.layout.dim.data[0].label.data);  //labelの長さを設定
  msg_pub.layout.dim.data[0].size = ANALOG_IN_CH; //  publishするメッセージの次元のサイズを設定
  msg_pub.layout.dim.data[0].stride = ANALOG_IN_CH; //　publishするメッセージの更新ストライドを設定
  //  メッセージ変数を初期化
  for (size_t i = 0; i < ANALOG_IN_CH; i++) {
    ain[i] = 0; //ain配列（アナログ入力の値）の初期化
    msg_pub.data.data[i] = ain[i];  //  メッセージのデータ部分に初期化された値を格納
  }
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    msg_sub.data.data[i] = 255; //アナログ出力データを初期化
    aout[i] = msg_sub.data.data[i]; //  アナログ出力データをaout配列にコピー
    
  }

  delay(2000);

  //****************************************
  // micro-ROS functions setup
  //****************************************

  // ①create default allocator（メモリ管理のためのアロケータ（メモリ確保用の管理機構）を取得）
  allocator = rcl_get_default_allocator();

  // ②create init_options（サポートオプションを設定。引数もコピペでよい）
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // ③create node（引数：(初期化するノード, ノードの名前, ノード空間の名前, サポート構造体)）
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //------create subscriber（subscriberの初期化）--------------------------------------------------------------------
  RCCHECK(rclc_subscription_init_default(
    &subscriber,  //  subscriberの構造体を指定
    &node,  //  subscriberが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), //  subscriberが受け取るメッセージの型の定義(std_msgs/msg/UInt16MultiArray 型)
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
  analogWrite(aout_channels[0], 140);
  analogWrite(aout_channels[1], 116);
  /*ピン2,3*/
  analogWrite(aout_channels[2], 128);
  analogWrite(aout_channels[3], 128);
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

  //  移動平均法1回目の処理(ポテンショメータ)
  for (int i = 0; i < LPF_kosuu; i++){
    Moving_LPF();
  }

  //==========================================================

  // turn off LED
  digitalWrite(LED, LOW);

  //------run the control thread（新しいスレッドの作成。thread_cakkback関数が繰り返される）--------------------------------------------------------------------
  threads.addThread(thread_callback);

}

void loop() {

  // spin the ros tasks（Executorを実行）
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
