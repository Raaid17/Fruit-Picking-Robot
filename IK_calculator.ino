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
double x=97, y=105, z=0, L1=105, L2=97;

int M1 = 90, M2 = 90, M3 = 170, input =0;

void setup() {
Serial.begin(9600);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("16 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  moveToPos( x, y, z, L1);
  
/*pwm.setPWM(0, 0, angleToPulse(M1) );
pwm.setPWM(1, 0, angleToPulse(M2) );
pwm.setPWM(2, 0, angleToPulse(M3) );*/

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
if (input=='4'){
    delay(20);
        x -=2;
        moveToPos( x, y, z, L1);       
      }
    else if (input=='2'){
   delay(20); 
        x +=2;
        moveToPos( x, y, z, L1);
    }
  else if (input=='1'){
   delay(20);      
        y -=2;
        moveToPos( x, y, z, L1);
    }
    else if (input=='3'){
   delay(20);     
        y +=2;
        moveToPos( x, y, z, L1);
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

int angleToPulse(double ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
  Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return (int)pulse;}

void moveToPos(double x, double y, double z, double L1) {
   double b = atan2(z,x) * (180 / 3.1415); // base angle

  double l = sqrt(x*x + z*z); // x and z extension 

  double h = sqrt (l*l + y*y);

  double phi = atan(y/l) * (180 / 3.1415);

  double theta = acos((h/2)/L1) * (180 / 3.1415);
  
  double a1 = phi + theta; // angle for first part of the arm
  double a2 = phi - theta; // angle for second part of the arm

  moveToAngle(b,a1,a2);
}

void moveToAngle(double b, double a1, double a2) {

  pwm.setPWM(1, 0, angleToPulse(a1));
  pwm.setPWM(2, 0, angleToPulse(a2+175));
}
