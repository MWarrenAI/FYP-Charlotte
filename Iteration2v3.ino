//Author: Mark Warren
//Company: Staffordshire University
//Date: 25/05/2023
//Purpose: Designed for prototype of Metal Detection Robot
//Version: 2.0
//Modifications: 
//-Turnleft/Turnright modified
//-Servo Timings
//Acknowledgments:

//SoftwareSerial.h
//Interrupt-driven receive and other improvements by ladyada http://ladyada.net
//Pin change interrupt macros by Paul Stoffregen http://www.pjrc.com
//20MHz processor support by Garrett Mace http://www.macetech.com
//ATmega1280/2560 support by Brett Hagman http://www.rougerobotics.com

//Servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
//Copyright (c) 2009 Michael Margolis.  All right reserved.

//ezButton.h
//Copyright (c) 2019, ArduinoGetStarted.com. All rights reserved.

//avr/sleep.h
//Copyright (c) 2002, 2004 Theodore A. Roth
//Copyright (c) 2004, 2007, 2008 Eric B. Weddington
//Copyright (c) 2005, 2006, 2007 Joerg Wunsch
//All rights reserved.

//WiFiClient.h - Library for Arduino Wifi shield. 
//Copyright (c) 2011-2014 Arduino.  All right reserved.

//ArduinoJson.h - https://arduinojson.org
//Copyright (c) [2023] [Benoit Blancon]

//ESP8266WiFi.h - esp8266 Wifi support.
//Based on WiFi.h from Arduino WiFi shield library.
//Copyright (c) 2011-2014 Arduino.  All right reserved.
//Modified by Ivan Grokhotkov, December 2014

//ESP8266 code originally made by Electronoobs
//Accelerometer code originally made by AZDelivery
//EzButton code originally made by ArduinoGetStarted.com
//day-6 code originally made by InventrKits
//Sleep Demo Serial originally made by MacSimski, D. Cuartielles and later modified by DojoDave
//Ultrasonic sensor code originally made by Pillole di Arduino

//Servo and ezbutton libraries included
#include <Servo.h>
#include <ezButton.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <avr/sleep.h>

//digital pins
int leftmot = 13;
int rightmot = 12;
#define buttonPin 10
#define onboardLED 13 //flashes when metal detected

//analog pins
#define metalPin A0
#define ZPin A4
#define YPin A2
#define XPin A1
//top
#define ECHO_PINT A6
#define TRIG_PINT A7
//left
#define ECHO_PINL A10
#define TRIG_PINL A11
//right
#define ECHO_PINR A14
#define TRIG_PINR A15

//Original code modified from MAIN_ESP8266 obtained from https://electronoobs.com/eng_arduino_tut177.php  
SoftwareSerial esp8266(14, 15);
String str;

//declarations
int gridx = 0;
int gridy = 5;
int metalhere = 600;
int avoidobstacle = 20;
int flag = 1;
int metalValue = 0;
int buttonValue = digitalRead(buttonPin);
//Sonar
int distancet = 0;
int distancel = 0;
int distancer = 0;
//Original code modified from GY-61 Accelerometer Sensor Module ENG obtained from https://www.az-delivery.de/en/products/gy-61-adxl335-beschleunigungssensor-3-axis-neigungswinkel-modul?variant=32256883785824 
//Accelerometer
uint16_t xValue = 0;
uint16_t yValue = 0;
uint16_t zValue = 0;
int xMin = 265;
int xMax = 395;

int yMin = 260;
int yMax = 351;

int zMin = 261;
int zMax = 393;

const int samples = 10;

int minefound = 0;

int mine1x = 0;
int mine1y = 0;
int mine2x = 0;
int mine2y = 0;
int mine3x = 0;
int mine3y = 0;
int mine4x = 0;
int mine4y = 0;
int mine5x = 0;
int mine5y = 0;
int mine6x = 0;
int mine6y = 0;

//Declaration of servo classes
Servo left, right;

//Original code modified from setDebounceTime() obtained from https://arduinogetstarted.com/tutorials/arduino-button-library 
//initialise button value
ezButton button(buttonPin);  

//Original code modified from Sleep Demo Serial obtained from https://playground.arduino.cc/Learning/ArduinoSleepCode/
//set speed of serial read (bytes per second)
void setup() {
  set_sleep_mode(SLEEP_MODE_STANDBY);  
  Serial.begin(115200);
  esp8266.begin(115200);
  delay(2000);
  pinMode(onboardLED, OUTPUT); //Metal Detector 
  pinMode(leftmot, OUTPUT);
  pinMode(rightmot, OUTPUT);
  left.attach(leftmot);
  right.attach(rightmot);
  
  //Sonars
  pinMode(TRIG_PINT, OUTPUT);
  pinMode(ECHO_PINT, INPUT);
  pinMode(TRIG_PINR, OUTPUT);
  pinMode(ECHO_PINR, INPUT);
  pinMode(TRIG_PINL, OUTPUT);
  pinMode(ECHO_PINL, INPUT);
  Serial.println("Press Button to move");
}

int leftMotorControl(int value){
  left.write(map(value,-100,100,1000,2000));
}
int rightMotorControl(int value){
  right.write(map(value,-100,100,1000,2000));
}

void sleeptime()
{
  delay(500);
  sleep_mode();
}

//Original code modified from Ultrasonic sensor: measure distances with Arduino obtained from https://pillolediarduino.altervista.org/en/ultrasonic-sensor-measure-distances-with-arduino/
void readfront()
{
  digitalWrite(TRIG_PINT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PINT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PINT, LOW);
  distancet = pulseIn(ECHO_PINT, HIGH);
  distancet = (distancet / 2) / 29.1; //cms
  Serial.print("Centimeters from top: ");
  Serial.println(distancet);
  str = String("Centimeters from top: (from arduino): ") + String(distancet);
  esp8266.println(str);
  delay(1000);
}

void readleft()
{
  digitalWrite(TRIG_PINL, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PINL, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PINL, LOW);
  distancel = pulseIn(ECHO_PINL, HIGH);
  distancel = (distancel / 2) / 29.1; //cms
  Serial.print("Centimeters from left: ");
  Serial.println(distancel);
  str = String("Centimeters from left: (from arduino): ") + String(distancel);
  esp8266.println(str);
  delay(1000);  
}

void readright()
{
  digitalWrite(TRIG_PINR, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PINR, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PINR, LOW);
  distancer = pulseIn(ECHO_PINR, HIGH);
  distancer = (distancer / 2) / 29.1; //cms
  Serial.print("Centimeters from right: ");
  Serial.println(distancer);
  str = String("Centimeters from right: (from arduino): ") + String(distancer);
  esp8266.println(str);
  delay(500);
}

//Original code modified from day-6 obtained from https://inventr.io/lessons/day-6/
void readmetal()
{
  metalValue = analogRead(metalPin);
  digitalWrite(onboardLED, HIGH);
  delay(metalValue);
  digitalWrite(onboardLED, LOW);
  delay(metalValue);
  Serial.print("Metal Value: ");
  Serial.println(metalValue);
  str = String("Metal Value (from arduino): ") + String(metalValue);
  esp8266.println(str);
  delay(metalValue);
}

//Original code modified from VEX Motors Controllers, Motors and Arduino Robot obtained from https://www.youtube.com/watch?v=N2LpwizZARw

void forwardsl(int pow)    
{
  leftMotorControl(pow);
}
void forwardsr(int pow)    
{
  rightMotorControl(pow);
}

void readxyz()
{
  int xRaw = 0;
  int yRaw = 0;
  int zRaw = 0;

  for (int i = 0; i < samples; i++)
  {
    xRaw += analogRead(XPin);
    yRaw += analogRead(YPin);
    zRaw += analogRead(ZPin);
  }

  xRaw/=samples;
  yRaw/=samples;
  zRaw/=samples;

  long xMilliG = map(xRaw, xMin, xMax, -1000, 1000);
  long yMilliG = map(yRaw, yMin, yMax, -1000, 1000);
  long zMilliG = map(zRaw, zMin, zMax, -1000, 1000); 

  float xAccel = xMilliG/1000;
  float yAccel = yMilliG/1000;
  float zAccel = zMilliG/1000;      
  //Accelerometer Readings
  xValue = xRaw;
  yValue = yRaw;
  zValue = zRaw;
  delay(200);  

  Serial.print("X Value: ");
  Serial.println(xValue);
  Serial.print("X Acceleration: ");
  Serial.println(xAccel);
  Serial.print("Y Value: ");
  Serial.println(yValue);
  Serial.print("Y Acceleration: ");
  Serial.println(yAccel);
  Serial.print("Z Value: ");
  Serial.println(zValue);
  Serial.print("Z Acceleration: ");
  Serial.println(zAccel);
  str = String("X Value (from arduino): ") + String(xValue);
  esp8266.println(str);
  str = String("X Acceleration (from arduino): ") + String(xAccel);
  esp8266.println(str); 
  str = String("Y Value (from arduino): ") + String(yValue);
  esp8266.println(str);
  str = String("Y Acceleration (from arduino): ") + String(yAccel);
  esp8266.println(str); 
  str = String("Z Value (from arduino): ") + String(zValue);
  esp8266.println(str);
  str = String("Z Acceleration (from arduino): ") + String(zAccel);
  esp8266.println(str);   
}

void readings()
{
  readfront();
  readleft();
  readright();
  readmetal();
  readxyz();
}

void danger()
{
  Serial.print("Program Interrupted - Obstacles too dangerous to navigate");
  str = String("Program Interrupted - Obstacles too dangerous to navigate");
  esp8266.println(str);
  delay(500);
  sleep_mode();  
}

void forwardsonce()
{
  forwardsl(60);
  forwardsr(-60);
  delay(425);
  forwardsl(0);
  forwardsr(0);
  readings();
  delay(1000);
}

void backwardsonce()
{
  forwardsl(-60);
  forwardsr(60);
  delay(425);
  forwardsl(0);
  forwardsr(0);
  readings(); 
  delay(1000);
}

void turnright()
{
  forwardsl(-60);
  forwardsr(-60);
  delay(1190);
  forwardsl(0);
  forwardsr(0);
  readings(); 
  delay(1000);
}

void turnleft()
{
  forwardsl(60);
  forwardsr(60);
  delay(1190);
  forwardsl(0);
  forwardsr(0);
  readings(); 
  delay(1000);
}

void flagthis()
{
  minefound = minefound + 1;
  Serial.print("Flag: ");
  Serial.println(flag);
  str = String("Flag State (from arduino): ") + String(flag);
  esp8266.println(str);
  flag++;
  if (minefound == 1)
  {
    mine1x = gridx;
    mine1y = gridy;
  }
  if (minefound == 2)
  {
    mine2x = gridx;
    mine2y = gridy;
  }
  if (minefound == 3)
  {
    mine3x = gridx;
    mine3y = gridy;
  }
  if (minefound == 4)
  {
    mine4x = gridx;
    mine4y = gridy;
  }
  if (minefound == 5)
  {
    mine5x = gridx;
    mine5y = gridy;
  }
  if (minefound == 6)
  {
    mine6x = gridx;
    mine6y = gridy;
  }
  if (minefound > 6)
  {
    Serial.print("Cannot store mine position");
    str = String("Cannot store mine position (from arduino)");
  }
  Serial.print("Grid X Value: ");
  Serial.println(gridx);
  Serial.print("Grid Y Value: ");
  Serial.println(gridy);
  str = String("Grid X Value (from arduino): ") + String(gridx);
  esp8266.println(str);
  str = String("Grid Y Value (from arduino): ") + String(gridy);
  esp8266.println(str); 
  Serial.println();
  delay(500);
}

void conclude()
{
  if (minefound == 0)
  {
    Serial.println("Program Complete- No mines found.");
    Serial.println();
    Serial.println();
    str = String("Program Complete- No mines found.(from arduino)");
    esp8266.println(str);
    sleeptime();
  }
  if (minefound < 2)
  {
    Serial.println("Program Complete...");
    str = String("Program Complete...(from arduino)");
    esp8266.println(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine1x); 
    Serial.print("Y: "); 
    Serial.print(mine1y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.println(str);
    str = String(mine1x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine1y);
    esp8266.print(str);
    sleeptime();
  }
  if (minefound < 3)
  {
    Serial.println("Program Complete...");
    str = String("Program Complete...(from arduino)");
    esp8266.println(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine1x); 
    Serial.print("Y: "); 
    Serial.print(mine1y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine1x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine1y);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine2x); 
    Serial.print("Y: "); 
    Serial.print(mine2y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine2x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine2y);
    esp8266.print(str);
    sleeptime();
  }
  if (minefound < 4)
  {
    Serial.println("Program Complete...");
    str = String("Program Complete...(from arduino)");
    esp8266.println(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine1x); 
    Serial.print("Y: "); 
    Serial.print(mine1y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine1x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine1y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine2x); 
    Serial.print("Y: "); 
    Serial.print(mine2y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine2x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine2y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine3x); 
    Serial.print("Y: "); 
    Serial.print(mine3y); 
    str = String("Mine was found at: ");
    esp8266.print(str);
    str = String("X: ");
    esp8266.println(str);
    str = String(mine3x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine3y);
    esp8266.print(str);
    sleeptime();
  }
  if (minefound < 5)
  {
    Serial.print("Program Complete...");
    str = String("Program Complete...(from arduino)");
    esp8266.println(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine1x); 
    Serial.print("Y: "); 
    Serial.print(mine1y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine1x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine1y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine2x); 
    Serial.print("Y: "); 
    Serial.print(mine2y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine2x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine2y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine3x); 
    Serial.print("Y: "); 
    Serial.print(mine3y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine3x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine3y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine4x); 
    Serial.print("Y: "); 
    Serial.print(mine4y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine4x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine4y);
    esp8266.print(str);
    sleeptime();
  }
  if (minefound < 6)
  {
    Serial.print("Program Complete...");
    str = String("Program Complete...(from arduino)");
    esp8266.println(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine1x); 
    Serial.print("Y: "); 
    Serial.print(mine1y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine1x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine1y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine2x); 
    Serial.print("Y: "); 
    Serial.print(mine2y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine2x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine2y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine3x); 
    Serial.print("Y: "); 
    Serial.print(mine3y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine3x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine3y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine4x); 
    Serial.print("Y: "); 
    Serial.print(mine4y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine4x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine4y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine5x); 
    Serial.print("Y: "); 
    Serial.print(mine5y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine5x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine5y);
    esp8266.print(str);
    sleeptime();
  }
  if (minefound == 6)
  {
    Serial.print("Program Complete...");
    str = String("Program Complete...(from arduino)");
    esp8266.println(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine1x); 
    Serial.print("Y: "); 
    Serial.print(mine1y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine1x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine1y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine2x); 
    Serial.print("Y: "); 
    Serial.print(mine2y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine2x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine2y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine3x); 
    Serial.print("Y: "); 
    Serial.print(mine3y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine3x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine3y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine4x); 
    Serial.print("Y: "); 
    Serial.print(mine4y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine4x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine4y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine5x); 
    Serial.print("Y: "); 
    Serial.print(mine5y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine5x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine5y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine6x); 
    Serial.print("Y: "); 
    Serial.print(mine6y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine6x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine6y);
    esp8266.print(str);
    sleeptime();
  }
  if (minefound > 6)
  {
    Serial.print("Program Complete...");
    str = String("Program Complete...(from arduino)");
    Serial.print("More than 6 Mines were found");
    str = String("More than 6 Mines were found(from arduino)");
    esp8266.println(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine1x); 
    Serial.print("Y: "); 
    Serial.print(mine1y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine1x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine1y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine2x); 
    Serial.print("Y: "); 
    Serial.print(mine2y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine2x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine2y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine3x); 
    Serial.print("Y: "); 
    Serial.print(mine3y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine3x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine3y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine4x); 
    Serial.print("Y: "); 
    Serial.print(mine4y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine4x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine4y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine5x); 
    Serial.print("Y: "); 
    Serial.print(mine5y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine5x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine5y);
    esp8266.print(str);
    Serial.println("Mine was found at: ");
    Serial.print("X: "); 
    Serial.print(mine6x); 
    Serial.print("Y: "); 
    Serial.print(mine6y); 
    str = String("Mine was found at: ");
    esp8266.println(str);
    str = String("X: ");
    esp8266.print(str);
    str = String(mine6x);
    esp8266.print(str);
    str = String("Y: ");
    esp8266.print(str);
    str = String(mine6y);
    esp8266.print(str);
    sleeptime();
  }
}

void obstacle1()
{
  turnleft();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  delay(1000);
  turnright();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx + 1;
  readings();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx + 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy + 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  turnright();
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx + 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
}

void obstacle2()
{
  turnleft();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  delay(1000);
  turnright();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy + 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy + 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  turnright();
  readings();
  delay(1000);
  turnright();
  if (metalValue > metalhere)
  { 
    gridx = gridx + 1;
    delay(500);
    flagthis();
  }
  forwardsonce();
  turnleft();
  turnleft();
  gridx = gridx - 1;
  readings();
  delay(1000);
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  forwardsonce();
  turnleft();
}

void obstacle3()
{
  turnleft();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  delay(1000);
  turnright();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  gridy = gridy + 1;
  delay(500);
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  forwardsonce();
  turnleft();
  turnleft();
}

void obstacle4()
{
  turnleft();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx + 1;
  delay(1000);
  turnright();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  turnright();
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
}

void obstacle5()
{
  turnleft();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx + 1;
  delay(1000);
  turnright();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  turnright();
  readings();
  gridx = gridx - 1;
  delay(1000);
  turnright();
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  readings();
  delay(1000);
  turnright();
  turnright();
}

void obstacle6()
{
  turnright();
  readings();
  delay(1000);
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  forwardsonce();
  gridy = gridy - 1;
  turnleft();
  readings();
  gridx = gridx - 1;
  delay(1000);
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  forwardsonce();
  turnright();
  turnright();
}

void obstacle7()
{
  turnright();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  turnleft();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  turnleft();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  gridy = gridy + 1;
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  forwardsonce();
  turnleft();
  turnleft();
}

void obstacle8()
{
  turnright();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  turnleft();
  readings();
  gridx = gridx + 1;
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  forwardsonce();
  turnleft();
  turnleft();
}

void obstacle9()
{
  turnright();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy + 1;
  delay(1000);
  turnleft();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  readings();
  delay(1000);
  turnleft();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy - 1;
  readings();
  delay(1000);
  turnright();
  if (distancet < avoidobstacle)
  { 
    danger();
  }  
}

void obstacle10()
{
  turnright();
  delay(1000);
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridy = gridy + 1;
  delay(1000);
  turnleft();
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  readings();
  delay(1000);
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  forwardsonce();
  gridx = gridx - 1;
  readings();
  gridy = gridy - 1;
  delay(1000);
  turnleft();
  if (distancet < avoidobstacle)
  { 
    danger();
  }
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  forwardsonce();
  readings();
  delay(1000);
  conclude();
}

void obstacle11()
{
  Serial.print("Robot found Obstruction at Grid 5, 0.");
  Serial.println();
  Serial.println();
  str = String("Robot found Obstruction at Grid 5, 0. (from arduino)");
  esp8266.println(str);
  conclude();
}

void gridup()
{
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  if (distancet < avoidobstacle)
  { 
    obstacle1();
  }
  forwardsonce(); //once
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle1();                
  }
  forwardsonce(); //twice
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle1();         
  }
  forwardsonce(); //three times
  gridx = gridx + 1;   
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle2();         
  }
  forwardsonce(); //four times
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle3();        
  }
  forwardsonce(); //five times
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
}

void griddown()
{
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  if (distancet < avoidobstacle)
  { 
    obstacle4(); 
  } 
  forwardsonce(); //once
  gridx = gridx - 1; 
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle4();                 
  }
  forwardsonce(); //twice
  gridx = gridx - 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle4();         
  }
  forwardsonce(); //three times
  gridx = gridx - 1;   
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle5();         
  }
  forwardsonce(); //four times
  gridx = gridx - 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle6();         
  }
  forwardsonce(); //five times
  gridx = gridx - 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
}

void gridup2()
{
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  if (distancet < avoidobstacle)
  { 
    obstacle9();
  }
  forwardsonce(); //once
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  { 
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle9();                
  }
  forwardsonce(); //twice
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle9();         
  }
  forwardsonce(); //three times
  gridx = gridx + 1;   
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle10();         
  }
  forwardsonce(); //four times
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
  if (distancet < avoidobstacle)
  {
    obstacle11();        
  }
  forwardsonce(); //five times
  gridx = gridx + 1; 
  if (metalValue > metalhere)
  {
    flagthis();
  }
  conclude();
}

void loop() 
{
  button.loop(); // MUST call the loop() function first
  //button reading
  buttonValue = digitalRead(buttonPin);
  Serial.print("Button State: ");
  Serial.println(buttonValue);
  str = String("Button State (from arduino): ") + String(buttonValue);
  esp8266.println(str);
  delay(1000);
  readings();
  
    //buttonpressed
  if (buttonValue == 0) 
  {
    readings(); 
    Serial.print("Button Sensor Value: ");
    Serial.print(buttonValue);
    str = String("Button Sensor Value (from arduino): ") + String(buttonValue);
    esp8266.println(str);
    Serial.println();
    gridup();
    turnleft();
    forwardsonce();
    gridy = gridy - 1;
    turnleft();    
    griddown();
    turnright();
    forwardsonce();
    gridy = gridy - 1;
    turnright();  
    gridup();
    turnleft();
    forwardsonce();
    gridy = gridy - 1;
    turnleft();    
    griddown();
    turnright();
    forwardsonce();
    gridy = gridy - 1;
    turnright();  
    gridup2();
  }
}