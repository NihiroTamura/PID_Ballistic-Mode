#define analogInPin A8
#define analogInPin2 A16
#define analogOutPin 4
#define analogOutPin2 7
#define Switch 17

const float kp = 0.1;

int sensorValue = 0;
int pwmValue = 0;
float pwmValueVoltage = 0;
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
  //outputValuePID = outputValue * kp;
  // アナログ出力値を変更する
  analogWrite(analogOutPin, outputValue);
  //analogWrite(analogOutPin2, outputValuePID);

  // 出力されたPWM信号を読む
  pwmValue = analogRead(analogInPin2);

  // 出力されたPWM信号を電圧に変換
  pwmValueVoltage = 3.3*pwmValue/1023;

  Serial.print("Sensor:");
  //Serial.print("sensor = ");
  Serial.println(sensorValue);
  //Serial.print(",");
  //Serial.print("OUTPUT:");
  //Serial.print("\t output = ");
  //Serial.print(outputValue);
  //Serial.print(",");
  //Serial.print("PID:");
  //Serial.print("\t PID = ");
  //Serial.println(outputValuePID);
  Serial.print("PWM電圧:");
  Serial.println(pwmValue);

  // アナログ->デジタル変換のために2ミリ秒のウェイトが必要
  delay(2);

}
