#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 // Minimum pulse length count (out of 4096)
#define SERVOMAX  575 // Maximum pulse length count (out of 4096)

// Global variables
double x = 97, y = 105, z = 0;
double L1 = 105, L2 = 97;

double M1 = 90, M2 = 90, M3 = 170; //Motor angle, adjusted to 90 degree triangle
int input = 0, stat =0;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with Bluetooth!");
  Serial.println("16 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.setPWM(0, 0, angleToPulse(M1));
  moveToPos(x, y, z, L1); // initialize pos of joint to 90 degree triangle

  delay(2000);
}

void loop() {
  if (Serial.available()) { // initialize BLE
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    input = SerialBT.read();
    Serial.write(input);
    Serial.println();
  }
if (stat == 0){ // if stat 0, control end-effector using x,y,z
  if (input == '4') {
    delay(10);
    x -= 3;
    moveToPos(x, y, z, L1);
  } else if (input == '2') {
    delay(10);
    x += 3;
    moveToPos(x, y, z, L1);
  } else if (input == '1') {
    delay(10);
    y -= 3;
    moveToPos(x, y, z, L1);
  } else if (input == '3') {
    delay(10);
    y += 3;
    moveToPos(x, y, z, L1);
  } else if (input == '5') {
    delay(10);
    pwm.setPWM(0, 0, angleToPulse(M1));
    M1 -= 2;
  } else if (input == '6') {
    delay(10);
    pwm.setPWM(0, 0, angleToPulse(M1));
    M1 += 2;
  } else if (input =='A'){ // change mode to manually control each joint
  stat = 1;
}
else if (input =='B'){ //change mode to IK
  stat = 0;
}
}
if(stat == 1){
  if (input == '2') {
    delay(10);
    pwm.setPWM(1, 0, angleToPulse(M2));
    M2 -= 2;
  } else if (input == '4') {
    delay(10);
    pwm.setPWM(1, 0, angleToPulse(M2));
    M2 += 2;
  } else if (input == '3') {
    delay(10);
   pwm.setPWM(2, 0, angleToPulse(M3));
    M3 -= 2;
  } else if (input == '1') {
    delay(10);
    pwm.setPWM(2, 0, angleToPulse(M3));
    M3 += 2;
  } else if (input == '5') {
    delay(10);
    pwm.setPWM(0, 0, angleToPulse(M1));
    M1 -= 2;
  } else if (input == '6') {
    delay(10);
    pwm.setPWM(0, 0, angleToPulse(M1));
    M1 += 2;
  } else if (input =='A'){
  stat = 1;
}
else if (input =='B'){
  stat = 0;
}
  }
  delay(20);
}

void moveToPos(double x, double y, double z, double L1) { 
  double a3 = acos((x * x + y * y - L1 * L1-L2*L2) / (2 * L1 * L2));
  double a2 = atan(x/y) - atan((L2*sin(a3)) / (L1+L2*cos(a3)));
  // Calculate the angles using inverse kinematics
  
  // Convert angles from radians to degrees
  
  a2 = a2 * (180 / PI);
  a3 = a3 * (180 / PI);

  // Move the servos to the calculated angles
  moveToAngle( a2, a3);

  // Print the current joint angles and coordinates
  Serial.print("Current Joint Angles - ");

  Serial.print(" | A2: ");
  Serial.print(a2);
  Serial.print(" | A3: ");
  Serial.println(a3);

  Serial.print("Current Coordinates - ");
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" | Y: ");
  Serial.print(y);
  Serial.print(" | Z: ");
  Serial.println(z);
}

void moveToAngle( double a2, double a3) {
  // Move the servos to the calculated angles

  pwm.setPWM(1, 0, angleToPulse(90-a2));
  pwm.setPWM(2, 0, angleToPulse(a3+85));
}

int angleToPulse(double ang) { //convert angle to pulse
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}
