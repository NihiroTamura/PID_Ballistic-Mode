#include <MsTimer2.h>

static int deg=0;
static int width=0;
static int count=0;

void time_count(void);

void setup(){
  pinMode(10, OUTPUT);
  MsTimer2::set(20, time_count);
  MsTimer2::start();
}

void loop(){
}

void time_count(void){
  if(count<180){
    deg=count;
    count++;
  }else{
    deg=0;
    count=0;
  }

  width=500+deg*10.5;

  digitalWrite(10, HIGH);
  delayMicroseconds(width);
  digitalWrite(10, LOW);
}