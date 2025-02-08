/*
 * random関数の動作確認用スケッチ
 * 今回は、randomSeedで乱数の初期化をする
 */

void setup() {
  // シリアルモニタ初期設定
  Serial.begin(9600);
  while(!Serial){  
  }

  // 乱数の初期化
  randomSeed( analogRead(A1) );

  // 乱数を30個生成してシリアルモニタに表示する
  for(uint8_t count=0; count<30; count++){
    Serial.println( random(10) );
  } 

}

void loop() {

}