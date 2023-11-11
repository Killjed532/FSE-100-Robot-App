//Servo Library
#include <Servo.h>
// SH1106 Library (OLED Screen) & Definitions
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
//MPU6050
#include <MPU6050.h>
MPU6050 mpu;

#define OLED_RESET 23
Adafruit_SH1106 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

//LCD
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16

//UVC LED
#define UVC_LED 24

//UltraSonic
#define TRIG_PIN 13      // Trigger pin of HC-SR04
#define ECHO_PIN 12      // Echo pin of HC-SR04
#define MAX_DISTANCE 5.0  // Maximum distance to consider (in cm). Adjust as needed.

//LineTracker
#define rightSensor 54 //A0
#define middleSensor 55 //A1
#define leftSensor 56 //A2

//Servo Motor Variables
int servoThreePos = 0; //Lower rear Servo all the way left pans from 9 to 3 counter clockwise
int servoFourPos = 0; // Upper rear Servo all the way down pans from 9 to 3 counter clockwise
Servo servoThree;
Servo servoFour;
int servoThreeIncrement = 10;
int servoFourIncrement = 30;
bool run = true;
// Define motor pins

#define PIN_Voltage  57 // A3
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3


// SH1106 OLED Screen Variables
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#if (SH1106_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SH1106.h!");
#endif



/*  Pant Tilt UV-C LED Function
*   Controls the tilt and traverse angles
*   of servo motors 3 & 4. Motors traverse 
*   at increments until they are both at 180.
*   At which point the loop ends.
*/

void printServoLEDPos() {
    display.clearDisplay();  // Clear the previous display
    Serial.println("PrintLEDPOS()");
    // Set the cursor to the current Y position
    display.setCursor(0, 10);
    
    // Print your data
    display.print("Servo Three: ");
    display.println(servoThreePos);
    display.print("Servo Four: ");
    display.println(servoFourPos);
    
    display.display(); // Update the display with new data
}
float readUltrasonicDistance() {
  Serial.println("readUltrasonicDistance()");
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.0344) / 2.0;
  
  return distance;
}
bool movementDetected(float distance){
  Serial.println("movementDetected()");
  if (distance <= 5.0) {
    display.println(distance);
    ucv_LED_Shutdown();
    Serial.println("Movement Detected");
    return false;
  }
  return true;
}
void ucv_LED_Shutdown(){
  Serial.println("ucv_LED_Shutdown()");
    Serial.println("UV-C OFF");  
    digitalWrite(UVC_LED, LOW); 
    display.setCursor(0, 0);
    display.clearDisplay();
    display.println("UV-C OFF");
    delay(1000);
    display.setCursor(0, 0);
    display.clearDisplay();
    //Reset values of Servo Pos and return
    servoThreePos = 0;
    servoFourPos = 0;
    servoThree.write(servoThreePos);
    servoFour.write(servoFourPos);
}

void panTiltLED() {
    do {
    Serial.println("panTiltLEDloop()");
    float distance = readUltrasonicDistance();
    bool movement = movementDetected(distance);
    if (!movement){
      Serial.println("false");
      distance = readUltrasonicDistance();
      movementDetected(distance);
      break;
    }
    digitalWrite(UVC_LED, HIGH);
    servoThree.write(servoThreePos);
    printServoLEDPos();
    
    if(servoThreePos == 180 && servoFourPos == 180 ){
      //Clear Display and Turn LED OFF
      ucv_LED_Shutdown();
      Serial.println("Break");
      break;
    }
    //increment ServoFour
    if (servoThreePos == 180) {
      //Increment ServoFour 30 degrees
        servoFourPos += servoFourIncrement;
        if (servoFourPos >= 180) {
            servoFourPos = 180;
        }
        servoFour.write(servoFourPos);
    }
    //Increment Servo 3
    servoThreePos += servoThreeIncrement;
    if (servoThreePos == 0) {
      //Increment ServoFour 30 degrees
        servoFourPos += servoFourIncrement;
        if (servoFourPos >= 180) {
            servoFourPos = 180;
        }
        servoFour.write(servoFourPos);
    }
    // Bounds check for servoThree
    if (servoThreePos > 180) {
        servoThreePos = 170;
        servoThreeIncrement = -10;
    } 
    else if (servoThreePos < 0){
        servoThreePos = 10;
        servoThreeIncrement = 10;
    }
    delay(500);
  } while (true);
}
//Function for MPU6050
void moveForward(int pause) {
  Serial.println("MoveForward()");
  display.setCursor(0, 0);
  display.println("Moving   : Forward");
  display.display();
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 177);
  analogWrite(PIN_Motor_PWMB, 172);
  delay(pause);
  display.clearDisplay();
}

void moveHome(int pause) {
  display.setCursor(0, 0);
  display.println("Moving   : Home");
  display.display();
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 178);
  analogWrite(PIN_Motor_PWMB, 172);
  delay(pause);
  display.clearDisplay();
}

void moveBackward() {
  display.setCursor(0, 0);
  display.println("Moving   : Backward");
  display.display();
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 165);
  analogWrite(PIN_Motor_PWMB, 176);
  delay(500);
  display.clearDisplay();
}
void turnAround() {
  display.setCursor(0, 0);
  display.println("Turning   : Around");
  display.display();
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 173);
  analogWrite(PIN_Motor_PWMB, 176);
  delay(1010);
  display.clearDisplay();
}
void turnRight() {
  display.setCursor(0, 0);
  display.println("Turning   : Right");
  display.display();
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 175);
  analogWrite(PIN_Motor_PWMB, 173);
  delay(575);
  display.clearDisplay();
}
void turnLeft() {
  display.setCursor(0, 0);
  display.println("Turning   : Left");
  display.display();
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 170);
  analogWrite(PIN_Motor_PWMB, 175);
  delay(570);
  display.clearDisplay();
}

void enterRoom(){
  turnLeft();
  stopMotors();
  delay(1000);

  moveForward(1200);
  stopMotors();
  delay(1000);

  turnAround();
  stopMotors();
  delay(1000);
  panTiltLED();
}

void exitRoom(){
  moveForward(1200);
  stopMotors();
  delay(1000);

  turnLeft();
  stopMotors();
  delay(1000);
}

void lastRoom(){
  moveForward(1300);
  stopMotors();
  delay(1000);

  turnRight();
  stopMotors();
  delay(1000);
}

void returnToBase(){
  moveHome(6250);
  stopMotors();
  delay(10000);

  turnAround();
  stopMotors();
  delay(1000);
}
void halfRightForward() {
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 170);
  analogWrite(PIN_Motor_PWMB, 150); 
}

void halfRightBackward() {
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 170);
  analogWrite(PIN_Motor_PWMB, 150);
}

void halfLeftForward() {
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 150);
  analogWrite(PIN_Motor_PWMB, 176); 
}

void halfLeftBackward() {
  digitalWrite(PIN_Motor_AIN_1, LOW);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 150);
  analogWrite(PIN_Motor_PWMB, 176);
}
void stopMotors() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
}
bool obstacleDetected(){
  Serial.println("ObstacleDetected()");
  float distance = readUltrasonicDistance();
  if (distance <= MAX_DISTANCE) {
    Serial.println(distance);
    stopMotors();
    delay(1000);
    turnAround();
    delay(1500);
    stopMotors();  // Finally, stop the robot.
    return true;
    
  }
  return false;
}
void printSensorVal(int leftSensorValue, int middleSensorValue, int rightSensorValue){
  display.clearDisplay();  // Clear the previous display

  // Set the cursor to the current Y position
  display.setCursor(0, 10);  
  display.print("Left Sensor    : ");
  display.println(leftSensorValue);
  display.print("Middle Sensor  : ");
  display.println(middleSensorValue);
  display.print("Right Sensor   : ");
  display.println(rightSensorValue);
  
  display.display(); // Update the display with new data
}

void motorLinetrackTest(){
  while(true){
    Serial.println("motorLineTrackTest()");
    unsigned long startTime = millis();  // Capture the start time
    
    obstacleDetected();
    if (millis() - startTime > 120000) {  // Check if 2 minutes have passed
      break;  // Exit the loop
    }

    int leftSensorValue = analogRead(leftSensor);
    int rightSensorValue = analogRead(rightSensor);
    int middleSensorValue = analogRead(middleSensor);

    printSensorVal(leftSensorValue, middleSensorValue, rightSensorValue);

    delay(5000);
    returnToBase();
    break;
    moveForward(1500);
    stopMotors();
    delay(500);

    enterRoom();

    exitRoom();

    moveForward(2400);
    stopMotors();
    delay(500);

    enterRoom();

    exitRoom();

    moveForward(2400);
    stopMotors();
    delay(500);

    enterRoom();

    lastRoom();

    returnToBase();
    break;
  }
  
}

//Gyro Function
void gyroCorrection(){
  while (true){
    int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  unsigned long startTime = millis();  // Capture the start time
  if (millis() - startTime > 120000) {  // Check if 2 minutes have passed
            break;  // Exit the loop
        }
  else {
    //Motor Test
    motorLinetrackTest();
    break;
    }
  }
}

void reInitScreen(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1.8);
  display.setCursor(0, 0);
}

void setup() {
  
  Wire.begin();
  Serial.begin(9600);
  //MPU6050 init
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  // Servo and Serial init
  servoThree.attach(44);
  servoThree.write(servoThreePos);
  servoFour.attach(46);
  servoFour.write(servoFourPos);
  
  //Display init
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display(); //Splash Screen
  delay(500);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1.8);
  display.setCursor(0, 0);
  
  //Pinmodes
  pinMode(UVC_LED, OUTPUT); //LED

  pinMode(PIN_Motor_PWMA, OUTPUT);//Track Motor
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);// Track Motor

  pinMode(TRIG_PIN, OUTPUT);//Ultrasonic
  pinMode(ECHO_PIN, INPUT);//Ultrasonic

  pinMode(leftSensor, INPUT);//LineTrack
  pinMode(rightSensor, INPUT);
  pinMode(middleSensor, INPUT);

  // Activate STBY mode
  digitalWrite(PIN_Motor_STBY, HIGH);
  Serial.println("Motor driver activated!");
}


void loop() {
  //Make sure LED is off in case of restart
  delay(1000);
  //Motor
  gyroCorrection();
  
  
  display.println("Reintitializing!");
 
  //printServoLEDPos();
  
}
