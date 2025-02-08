#include <TeensyThreads.h>

volatile long old[4];

void one() {
  while (1) {
    Serial.print("ONE ");
    Serial.print(threads.id());
    Serial.print(" ");
    Serial.println(millis());
    threads.delay(1000);
  }
}

void two() {
  while (1) {
    Serial.print("TWO ");
    Serial.print(threads.id());
    Serial.print(" ");
    Serial.println(millis());
    threads.delay(1000);
  }
}

void three() {
  while (1) {
    Serial.print("THREE ");
    Serial.print(threads.id());
    Serial.print(" ");
    Serial.println(millis());
    threads.delay(1000);
  }
}

void setup() {
  Serial.begin(9600);
  threads.addThread(one);
  threads.addThread(two);
  threads.addThread(three);
}

void loop() {
  Serial.print("MAIN ");
  Serial.print(threads.id());
  Serial.print(" ");
  Serial.println(millis());
  threads.delay(1000);
//  delay(1000);   //上の行をコメントアウトしてこの行のコメントアウトを外すと1秒間隔がずれていく
}
