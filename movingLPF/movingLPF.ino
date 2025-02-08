#include <queue>

#define InputPin A17

#define kosuu 100

std::queue<int> q;

int count = 0;
int pot1 = 0;
int pot1_sum = 0;
int pot1_last = 0;
int POT1 = 0;


void setup() {
  Serial.begin(9600);
}

void loop() {
  POT1 = Moving_LPF();
  Serial.println(POT1);
}

int Moving_LPF() {
  // キューに値を貯め込む
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
