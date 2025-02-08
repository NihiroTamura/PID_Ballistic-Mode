#define analogInPin A6
#define analogOutPin 4
#define analogOutPin2 7
#define Switch 22

const float kp = 0.1;

int sensorValue = 0;
int outputValue = 0;
int outputValuePID = 0;

void setup() {
  //シリアル通信を初期化する。ボーレートは9600bps
  Serial.begin(9600);

  //スイッチのモード設定
  pinMode(Switch, INPUT_PULLUP);

  while(digitalRead(Switch) == 1){
    //何もしない処理
  }
}

void loop() {
  // アナログ入力の値を読む
  sensorValue = analogRead(analogInPin);
  // アナログ入力の0～1023の値を、アナログ出力の使用範囲0～255に変換する
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  // アナログ出力値にPIDゲインをかける
  outputValuePID = outputValue * kp;
  // アナログ出力値を変更する
  analogWrite(analogOutPin, outputValue);
  analogWrite(analogOutPin2, outputValuePID);

  Serial.print("Sensor:");
  //Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print(",");
  Serial.print("OUTPUT:");
  //Serial.print("\t output = ");
  Serial.print(outputValue);
  Serial.print(",");
  Serial.print("PID:");
  //Serial.print("\t PID = ");
  Serial.println(outputValuePID);

  // アナログ->デジタル変換のために2ミリ秒のウェイトが必要
  delay(2);

}
