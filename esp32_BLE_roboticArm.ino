#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> 
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
//uint8_t servonum = 0;

int M1 = 90, M2 = 90, M3 = 170, input =0;

void setup() {
Serial.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("16 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates


  
pwm.setPWM(0, 0, angleToPulse(M1) );
pwm.setPWM(1, 0, angleToPulse(M2) );
pwm.setPWM(2, 0, angleToPulse(M3) );

delay(2000);

 
}

void loop() {

   
 if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
   input = SerialBT.read();
   Serial.write(input);
   Serial.println();

    }
if (input=='2'){
    delay(20);
    pwm.setPWM(0, 0, angleToPulse(M1));
        M1 -=2;
          
      }
    else if (input=='4'){
   delay(20);
        pwm.setPWM(0, 0, angleToPulse(M1));
        M1 +=2;
    }
  else if (input=='3'){
   delay(20);
        pwm.setPWM(1, 0, angleToPulse(M2));
        M2 -=2;
    }
    else if (input=='1'){
   delay(20);
        pwm.setPWM(1, 0, angleToPulse(M2));
        M2 +=2;
    }
    else if (input=='5'){
   delay(20);
        pwm.setPWM(2, 0, angleToPulse(M3));
        M3 -=2;
    }
    else if (input=='6'){
   delay(20);
        pwm.setPWM(2, 0, angleToPulse(M3));
        M3 +=2;
    }
  delay(20);    
}

int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
  Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;}
