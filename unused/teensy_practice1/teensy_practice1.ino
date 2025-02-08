/*LEDを2つ光らせるプログラム*/

#define PWM 4
#define PWM2 9
#define LED_JIKAN 1000
#define LED_JIKAN_x2 LED_JIKAN*2

void setup(){
  pinMode(PWM , OUTPUT);
  pinMode(PWM2, OUTPUT);
}

void loop()
{
  digitalWrite(PWM, HIGH);
  digitalWrite(PWM2, LOW);
  delay(LED_JIKAN);
  digitalWrite(PWM, LOW);
  digitalWrite(PWM2, HIGH);
  delay(LED_JIKAN);
  digitalWrite(PWM, HIGH);
  digitalWrite(PWM2, LOW);
  delay(LED_JIKAN);
  digitalWrite(PWM, LOW);
  digitalWrite(PWM2, HIGH);
  delay(LED_JIKAN);
  digitalWrite(PWM, HIGH);
  digitalWrite(PWM2, LOW);
  delay(LED_JIKAN_x2);
  digitalWrite(PWM, LOW);
  digitalWrite(PWM2, HIGH);
  delay(LED_JIKAN_x2);
}