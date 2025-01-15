//=============================================================================================================================================

//　制御周期(単位：ms)
#define CONTROL_PERIOD_MS 1

//　ROSのPublishする周期(単位：ms)
#define PUB_PERIOD_MS 10

//　LEDの定義ピン
#define LED 13

// Topic names
#define SUB_TOPICNAME "/board1/aout"
#define PUB_TOPICNAME "/board1/ain"

// AD/DA conv channels
//　アナログ入力のピン数
#define ANALOG_IN_CH 18

//　アナログ出力のピン数
#define ANALOG_OUT_CH 12


//　アナログ入出力のピンを定義
const int ain_channels[ANALOG_IN_CH] = {14,15,16,17,18,19,20,21,22,23,24,25,26,27,38,39,40,41};
const int aout_channels[ANALOG_OUT_CH] = {0,1,2,3,4,5,6,7,8,9,28,29};

//=============================================================================================================================================

//------ライブラリのインクルード--------------------------------------------------------------------
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include "TeensyThreads.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/u_int16_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

//------マクロの定義--------------------------------------------------------------------
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// ROS2の関数fnが成功したかどうかを確認。成功したら何もせず次の処理に進み、失敗したらerror_loop()を呼び出しエラー状態にする

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// ROS2の関数fnが成功したかどうかを確認。成功したら何もせず次の処理に進み、失敗しても何もせずスルーする

//------オブジェクトの定義--------------------------------------------------------------------
std_msgs__msg__UInt16MultiArray msg_pub;
std_msgs__msg__UInt16MultiArray msg_sub;

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//------グローバル変数の定義--------------------------------------------------------------------
//　アナログ出力の配列
volatile uint16_t aout[12];
//　アナログ入力の配列
volatile uint16_t ain[18];

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

      for(int i = 0; i < ANALOG_IN_CH; i++){
        ain[i] = analogRead(ain_channels[i]);
      }

      //
      //
      //
    
      for(int i = 0; i < ANALOG_OUT_CH; i++){
        analogWrite(aout_channels[i], aout[i]);
      }
    }
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

//=============================================================================================================================================

void setup() {
  //　micro-rosの通信手段を設定
  set_microros_transports();
  
  // configure LED pin
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // configure PWM/AD pins
  for (size_t i = 0; i < ANALOG_OUT_CH; i++) {
    pinMode(aout_channels[i], OUTPUT);
  }
  for (size_t i = 0; i < ANALOG_IN_CH; i++) {
    pinMode(ain_channels[i], INPUT);
  }

  // allocate message variables（pubもsubも1行目の右辺のみ変更可。その他はコピペ）
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

  // initialize message variables
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

  // create subscriber（subscriberの初期化）
  RCCHECK(rclc_subscription_init_default(
    &subscriber,  //  subscriberの構造体を指定
    &node,  //  subscriberが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), //  subscriberが受け取るメッセージの型の定義(std_msgs/msg/UInt16MultiArray 型)
    SUB_TOPICNAME));  //  subscribeするトピックの名前

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher, //  publisherの構造体を指定
    &node,  //  publisherが関連付けられるノードを指定
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray), //  publishするメッセージの型の定義(std_msgs/msg/UInt16MultiArray 型)
    PUB_TOPICNAME));  //  publishするトピックの名前

  // create timer（timerを利用して定期的に実行するtimer_callback関数を設定）
  const unsigned int timer_timeout = PUB_PERIOD_MS;
  RCCHECK(rclc_timer_init_default(
    &timer, //  タイマー構造体
    &support, //  サポート構造体
    RCL_MS_TO_NS(timer_timeout),  //  周期を指定
    timer_callback)); //  コールバック関数の指定

  // create executor
  //  Executor（実行管理エンジン）を初期化（引数：(初期化するExecutor構造体, Executorが利用するROS2のコンテキスト, Executorが管理するハンドル（ex. subscribe,timer）の数, メモリ割り当て用のカスタムアロケータ)）
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  //  SubscriberをExecutorに追加（引数：(初期化済みのExecutor, 追加するsubscriber構造体, subscriberが受信したメッセージ変数, subscriber用コールバック関数, 新しいデータが届いたときのみコールバックを実行する動作モード)）
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));
  //  timerをExecutorに追加（引数：(初期化済みのExecutor, 追加するtimer構造体)）
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // turn off LED
  digitalWrite(LED, LOW);

  // run the control thread（新しいスレッドの作成。thread_cakkback関数が繰り返される）
  threads.addThread(thread_callback);
}

void loop() {

  // spin the ros tasks（Executorを実行）
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
