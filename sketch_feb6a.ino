#include <Wire.h>
#include "Adafruit_TCS34725.h"
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
#define echoPin 42 
#define trigPin 44
long duration; 
int distance;
String msg;
int msg0;
int klvye;
bool kas0;
byte gammatable[256];


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

uint8_t servonum = 0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  klvye = 0;
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  //Serial.println("Color View Test!");

  if (tcs.begin()) {
    //Serial.println("Found sensor")taaaaa
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
  }
//


pwm.begin();
pwm.setPWMFreq(FREQUENCY);
}
int pulseWidth(int angle)
{
int pulse_wide, analog_value;
pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
Serial.println(analog_value);
return analog_value;

}

void loop() {

  
  if (analogRead(A8) >30) {
    analogWrite(50,130);
  } else if (analogRead(A8) > 60) {
    analogWrite(50,170);
  } else if (analogRead(A8)  >= 90) {
    analogWrite(50,255);
  }else{
  analogWrite(50,0);
  }
  if (analogRead(A9) >30) {
    analogWrite(48,130);
  } else if (analogRead(A9) > 60) {
    analogWrite(48,170);
  } else if (analogRead(A9)  >= 90) {
    analogWrite(48,255);
  } else{
   analogWrite(48,0);
  }
  if (analogRead(A10) >100) {
    analogWrite(46,130);
  } else if (analogRead(A10) > 30) {
    analogWrite(46,170);
  } else if (analogRead(A10)  >=90) {
    analogWrite(46,255);
  }else{
    analogWrite(46,0);
  }



  float red, green, blue;
  readSerialPort();

  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  //Serial.println(distance);
  if(distance <= 5)
  {
  if(blue >= green && blue >= green ){
    sendData0();
    //pwm.setPWM(0, 0, pulseWidth(0));
    }else if(red >= green && red >= blue ){
      sendData1();
      } else if(green >= red && green >= blue){
        sendData2();
        
        }

  }


#if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(1, gammatable[(int)red]);
  ledcWrite(2, gammatable[(int)green]);
  ledcWrite(3, gammatable[(int)blue]);
#else
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
#endif
}


void readSerialPort() {
  msg = "";
  if (Serial.available()) {
    delay(10);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
    }
    Serial.flush();
  }
  if(analogRead(A1)> 550)//100 geçiçi
  {
   kas0 = true;

  }else{
    kas0 = false;
  }
  /*
  if(analogRead(A0)> 100)//100 geçiçi
  {
   kas0 = true;

  }else{
    kas0 = false;
  }
*/

  msg0 = msg.toInt();
  if(msg0 ==  0){
    pwm.setPWM(9, 0, pulseWidth(100));
  }else if (msg0 == 1 ){
    pwm.setPWM(9, 0, pulseWidth(0));
  }else if (msg0 == 2 ){
    pwm.setPWM(9, 0, pulseWidth(120));
  }else if (msg0 == 3 ){
  pwm.setPWM(8, 0, pulseWidth(180)); 
  }else if (msg0 == 4 ){
  pwm.setPWM(8, 0, pulseWidth(120)); 
  pwm.setPWM(8, 0, pulseWidth(120)); 
  }else if (msg0 == 5 ){//top
    if( kas0 == true)
    {
      pwm.setPWM(7, 0, pulseWidth(60));
      pwm.setPWM(4, 0, pulseWidth(90));
      pwm.setPWM(5, 0, pulseWidth(115));

      pwm.setPWM(0, 0, pulseWidth(90));
      pwm.setPWM(1, 0, pulseWidth(90));
      pwm.setPWM(2, 0, pulseWidth(90));
      pwm.setPWM(3, 0, pulseWidth(90));
      pwm.setPWM(6, 0, pulseWidth(90));
      
    }else{
      pwm.setPWM(7, 0, pulseWidth(60));
      pwm.setPWM(4, 0, pulseWidth(90));
      pwm.setPWM(5, 0, pulseWidth(115));

      pwm.setPWM(0, 0, pulseWidth(0));
      pwm.setPWM(1, 0, pulseWidth(0));
      pwm.setPWM(2, 0, pulseWidth(180));
      pwm.setPWM(3, 0, pulseWidth(180));
      pwm.setPWM(6, 0, pulseWidth(0));
    }
  }else if (msg0 == 6 ){//sose bottle
    if( kas0 == true)
    {
      pwm.setPWM(7, 0, pulseWidth(130));
      pwm.setPWM(4, 0, pulseWidth(180));
      pwm.setPWM(5, 0, pulseWidth(0));

      pwm.setPWM(0, 0, pulseWidth(90));
      pwm.setPWM(1, 0, pulseWidth(90));
      pwm.setPWM(2, 0, pulseWidth(90));
      pwm.setPWM(3, 0, pulseWidth(90));
      pwm.setPWM(6, 0, pulseWidth(180));
      
    }else{
      pwm.setPWM(7, 0, pulseWidth(130));
      pwm.setPWM(4, 0, pulseWidth(180));
      pwm.setPWM(5, 0, pulseWidth(0));

      pwm.setPWM(0, 0, pulseWidth(0));
      pwm.setPWM(1, 0, pulseWidth(0));
      pwm.setPWM(2, 0, pulseWidth(180));
      pwm.setPWM(3, 0, pulseWidth(180));
      pwm.setPWM(6, 0, pulseWidth(0));
    }
  }else if (msg0 == 7 ){//mouse
     pwm.setPWM(3, 0, pulseWidth(145));
     pwm.setPWM(2, 0, pulseWidth(145));

     pwm.setPWM(4, 0, pulseWidth(145));
     pwm.setPWM(5, 0, pulseWidth(0));
     pwm.setPWM(6, 0, pulseWidth(60));

    if( kas0 == true )
    {
      pwm.setPWM(0, 0, pulseWidth(30));
      pwm.setPWM(1, 0, pulseWidth(30));
    }else{
    pwm.setPWM(0, 0, pulseWidth(0));
      pwm.setPWM(1, 0, pulseWidth(0));

    }
     if( kas0 == true )
    {
      pwm.setPWM(0, 0, pulseWidth(30));
      pwm.setPWM(1, 0, pulseWidth(30));
    }else{
    pwm.setPWM(0, 0, pulseWidth(0));
    pwm.setPWM(1, 0, pulseWidth(0));
      
    }
  }else if (msg0 == 8 ){//keyboard 
    pwm.setPWM(7, 0, pulseWidth(60));
    pwm.setPWM(5, 0, pulseWidth(115));
    pwm.setPWM(4, 0, pulseWidth(90));
    if( kas0 == true)
    {
      klvye +=1;
    }

    if(klvye >= 5){
     klvye = 0;
    }
    if( kas0 == true)
    {
      if(klvye <=3)
      {
        pwm.setPWM(klvye, 0, pulseWidth(60));
        delay(10);
        pwm.setPWM(klvye, 0, pulseWidth(0));
      }else if(klvye == 4){
        pwm.setPWM(6, 0, pulseWidth(60));
        delay(10);
        pwm.setPWM(6, 0, pulseWidth(0));

      }
    }
  }else if (msg0 == 9 ){//daha fazla grip ekle
    if( kas0 == true)
    {

    }else{

      
    }
  }

}

void sendData0() {
  Serial.println("a");
}
void sendData1() {
  Serial.println("b");
}
void sendData2() {
  Serial.println("c");
}
