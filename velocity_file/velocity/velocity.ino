#include <queue>
#define InputPin A17

#define kosuu 1500
std::queue<int> q;
int count = 0;
int pot1 = 0;
int pot1_sum = 0;
int pot1_last = 0;
int POT1 = 0;


/*-------*/

int initial_ps = 0;
const int windowSize = 7;
float delta_h = 0.0; // サンプリング間隔（秒）
float derivative = 0.0;
// 微分係数（7点中心差分法）
const float coeffs[7] = { -1.0 / 60, 3.0 / 20, -3.0 / 4, 0.0, 3.0 / 4, -3.0 / 20, 1.0 / 60 };
int pt = 0;
float SPEED1 = 0.0;

#define kosuu_sample 19 // サンプリング間隔（個数）

std::queue<int> potValues;

struct Result_POT_Speed {
  int POT;
  float Speed;
};

/*-------*/
struct Result_LPF_SPEED{
  int POT_1;
  float Speed_1;
};

#define kosuu_speed 3000
std::queue<float> q_speed;

int count_speed = 0;
float speed1_sum = 0;
float speed1 = 0;
float speed1_last = 0;


void setup() {
  Serial.begin(9600);
}

void loop() {
  unsigned long TS, TE, FREQ;

  TS = micros();

  //POTの値とSpeedを取得
  Result_LPF_SPEED PS_1 = Moving_LPF_SPEED();
  POT1 = PS_1.POT_1;
  SPEED1 = PS_1.Speed_1;


  TE = micros();
  FREQ = (TE - TS) ; // サンプリング間隔（秒）を計算
  
  // 結果をシリアルモニタに出力
  //Serial.print("Derivative: ");
  Serial.print(POT1);
  Serial.print("\t ,");
  Serial.print(SPEED1);
  Serial.print("\t ,");
  Serial.print(FREQ);
  Serial.print("\t ,");
  Serial.println(speed1_sum);

}

int Moving_LPF() {
  // 移動平均フィルタ
  if (count == 0) {
    for (int i = 0; i < kosuu; i++) {
      int analog_value = analogRead(InputPin); // アナログ値を1回だけ読み取る
      q.push(analog_value);                    // キューに追加
      pot1_sum += analog_value;                // 合計を更新
    }
    pot1 = pot1_sum / kosuu; // 初期の平均値を計算

  } else {
    pot1_last = q.front(); // 最初の値を取得
    q.pop();               // 最初の値を削除
    pot1_sum -= pot1_last; // 合計から削除した値を引く

    int analog_value = analogRead(InputPin); // 新しいアナログ値を1回だけ読み取る
    q.push(analog_value);                    // キューに追加
    pot1_sum += analog_value;                // 合計に加算

    pot1 = pot1_sum / kosuu; // 新しい平均値を計算
  }

  count = 1; // 初期化が完了したことを記録
  return pot1;
}

Result_POT_Speed POT_Speed(){
  Result_POT_Speed ps;

  unsigned long ts, te;

  //初回
  if(initial_ps == 0){

    //7点取得
    for(int i = 0; i < windowSize; i++){
      // サンプリング間隔を計測スタート
      ts = micros();

      //POTの値取得
      int analog_pot1 = Moving_LPF();
      potValues.push(analog_pot1);

      // kosuu_sample個分の値をスキップ
      for (int i = 0; i < kosuu_sample; i++) {
        Moving_LPF(); // スキップのみ
      }

      // サンプリング間隔を計測ストップ
      te = micros();
      delta_h = (te - ts) * 1e-6; // サンプリング間隔（秒）を計算

    }

    // 微分値を計算
    if (potValues.size() == windowSize) {
      derivative = 0.0; // 毎ループでリセット
      std::queue<int> tempQueue = potValues; // キューをコピー

      // 最新の値を取り出して計算
      for (int i = 0; i < windowSize; i++) {
        derivative += coeffs[i] * tempQueue.front();

        if(i == 3){
          pt = tempQueue.front();
        }

        tempQueue.pop();
      }

      // Δhを考慮して微分値を計算
      derivative /= delta_h;
    }

  }else{
    // サンプリング間隔を計測スタート
    ts = micros();

    //POTの値取得
    int analog_pot1 = Moving_LPF();
    potValues.push(analog_pot1);

    // kosuu_sample個分の値をスキップ
    for (int i = 0; i < kosuu_sample; i++) {
      Moving_LPF(); // スキップのみ
    }

    // キューが最大サイズを超えたら古い値を削除
    if (potValues.size() > windowSize) {
      potValues.pop();
    }

    // サンプリング間隔を計測ストップ
    te = micros();
    delta_h = (te - ts) * 1e-6; // サンプリング間隔（秒）を計算

    // 微分値を計算
    if (potValues.size() == windowSize) {
      derivative = 0.0; // 毎ループでリセット
      std::queue<int> tempQueue = potValues; // キューをコピー

      // 最新の値を取り出して計算
      for (int i = 0; i < windowSize; i++) {
        derivative += coeffs[i] * tempQueue.front();

        if(i == 3){
          pt = tempQueue.front();
        }

        tempQueue.pop();
      }

      // Δhを考慮して微分値を計算
      derivative /= delta_h;
    }
  }

  ps.POT = pt;
  ps.Speed = derivative;

  
  /*Serial.print(1/delta_h, 6); // Δhを秒単位で表示（小数点以下6桁）
  Serial.print("\t ,");
  Serial.print(delta_h, 6);
  Serial.print("\t ,");*/

  initial_ps = 1;

  return ps;
}

Result_LPF_SPEED Moving_LPF_SPEED() {
  Result_LPF_SPEED PPSS;

  int pot1_ = 0;
  float speed1_ = 0;

  // 移動平均フィルタ
  if (count_speed == 0) {
    for (int i = 0; i < kosuu_speed; i++) {
      Result_POT_Speed PS = POT_Speed();
      pot1_ = PS.POT;
      speed1_ = PS.Speed;

      q_speed.push(speed1_);

      speed1_sum += speed1_;                // 合計を更新
    }
    speed1 = speed1_sum / kosuu_speed; // 初期の平均値を計算

  } else {
    speed1_last = q_speed.front(); // 最初の値を取得
    q_speed.pop();               // 最初の値を削除
    speed1_sum -= speed1_last; // 合計から削除した値を引く

    Result_POT_Speed PS = POT_Speed();
    pot1_  = PS.POT;
    speed1_ = PS.Speed;

    q_speed.push(speed1_);
    speed1_sum += speed1_;                // 合計に加算

    speed1 = speed1_sum / kosuu_speed; // 新しい平均値を計算
  }

  count_speed = 1; // 初期化が完了したことを記録

  PPSS.POT_1 = pot1_;
  PPSS.Speed_1 = speed1_;


  return PPSS;
}