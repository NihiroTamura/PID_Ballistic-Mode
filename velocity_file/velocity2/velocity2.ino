#include <queue>

#define InputPin A17
#define kosuu 1500

std::queue<int> q;

int count = 0;
int pot1 = 0;
int pot1_sum = 0;
int pot1_last = 0;
int POT1 = 0;

const float coef_lpf = 0.99;
int df_filter = 0;
int previous_derivative = 0;
int initial_lpf = 0;

// キューの最大サイズ
const int windowSize = 7;
std::queue<int> potValues;
float derivative = 0.0;

// 微分係数（7点中心差分法）
const float coeffs[7] = { -1.0 / 60, 3.0 / 20, -3.0 / 4, 0.0, 3.0 / 4, -3.0 / 20, 1.0 / 60 };

float delta_h = 0.0; // サンプリング間隔（秒）

#define jk 19

void setup() {
  Serial.begin(9600);

  // 初期化時にキューに7点の値を準備
  for (int i = 0; i < windowSize; i++) {
    int POT1_1_1 = Moving_LPF();
    potValues.push(POT1_1_1);

    // 100個分の値をスキップ
    for (int j = 0; j < jk; j++) {
      Moving_LPF(); // スキップのみ
    }
  }
}

void loop() {
  unsigned long ts, te, TE, TS, time;

  // サンプリング間隔を計測
  TS = micros();
  ts = micros();

  // POT1を取得してキューに追加
  int POT1_1 = Moving_LPF();
  potValues.push(POT1_1);

  // 100個分の値をスキップ
  for (int i = 0; i < jk; i++) {
      Moving_LPF(); // スキップのみ
  }

  // キューが最大サイズを超えたら古い値を削除
  if (potValues.size() > windowSize) {
    potValues.pop();
  }


  te = micros();
  delta_h = (te - ts) * 1e-6; // サンプリング間隔（秒）を計算

  // 微分値を計算
  if (potValues.size() == windowSize) {
    derivative = 0.0;
    std::queue<int> tempQueue = potValues; // キューをコピー

    // 最新の値を取り出して計算
    for (int i = 0; i < windowSize; i++) {
      derivative += coeffs[i] * tempQueue.front();
      if(i == 3){
        POT1 = tempQueue.front();
      }
      tempQueue.pop();
    }

    // Δhを考慮して微分値を計算
    derivative /= delta_h;
  }

  //RCローパスフィルタ適用(VEAB)
  df_filter = RC_LPF(derivative, previous_derivative, initial_lpf, coef_lpf);

  initial_lpf += 1;

  //PWM値に格納
  derivative = df_filter;

  //前回のVEAB値に格納
  previous_derivative = df_filter;
  
  TE = micros();
  time = TE-TS;
  

  // 結果をシリアルモニタに出力
  //Serial.print("Derivative: ");
  Serial.print(POT1);
  Serial.print("\t ,");
  Serial.print(derivative);
  //Serial.print("Delta h (s): ");
  //Serial.print(delta_h, 6); // Δhを秒単位で表示（小数点以下6桁）
  Serial.print("\t ,");
  Serial.println(time);
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

//ローパスフィルタ(RCフィルタ)
float RC_LPF(float value, float previous_value, int initial_lpf, float coef_lpf){
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