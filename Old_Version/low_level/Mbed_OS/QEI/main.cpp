#include <mbed.h>
#include <QEI.h>
#include <math.h>
#define cutoff 3000
int input[3010];
int output[3010];
float pos,wave;
float INPUT_VOLTAGE = 12.0;
float t = 0.000;
float sampleTime = 1.0/100.0;
float speed;
int num = 0;
int num2 = 0;
int set = 0;
int a,b;
PwmOut pwm(D10);

DigitalOut motorA1(D7);
DigitalOut motorA2(D8);

Serial pc(USBTX, USBRX);

QEI qei(PA_15, PB_7, NC, 512);

Ticker interrupt;

void motor(float speed){
  if(speed < 0){
    motorA1 = 1;
    motorA2 = 0;
    
  }
  else if(speed > 0){
    motorA1 = 0;
    motorA2 = 1;
  }
  else{
    motorA1 = 0;
    motorA2 = 0;
  }
  pwm.write(abs(speed));
}

float chirpSine(float time){
  float signal = sin((0.1*(time*time)) + (3.14159265/2.0)) * INPUT_VOLTAGE;
  return signal;
}

float convertVolttoPwm(float volt){
  return (volt) / (INPUT_VOLTAGE);
}

void get_qei(){

  if (set == 1)
  {
    if (num2 < cutoff){
      pc.printf("%d \t %d\n", input[num2],output[num2] );
      num2++ ;
    }
    
  }
  else
  {
  wave = chirpSine(t);
  pos = qei.getPulses();
  pos = pos*2*3.14159265/(64.0*512.0);
  speed = convertVolttoPwm(wave);
  // speed = m;
  motor(speed);
  t = t + (sampleTime);
  a = (int)(wave*1000);
  b = (int)(pos*1000);
  input[num] = a;
  output[num] = b;
  num++;
  }
  
}

int main() {
  // pwm.period(1/100);
  // // put your setup code here, to run once:
  // pwm.period_us(10000000);
  pwm.period(1.0/500.0);
  interrupt.attach(&get_qei, sampleTime);
  while(1) {
  if (num >= cutoff)
  {
    set = 1;
    pwm.write(abs(0));
  }
  pc.printf("");
  }
  // pc.printf("%d \n", num);
  // pc.printf("%d \t %d\n", a, b);
        
    // get_qei();
    // pc.printf("%f \n", speed);
    // pc.printf("%f \t %f\n", wave, pos);
    // put your main code here, to run repeatedly:
  }
