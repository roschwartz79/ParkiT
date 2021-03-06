/*
  ParkIT peripheral sensor script - ECE SENIOR DESIGN PROJECT
  DESIGNED BY: ROB SCHWARTZ, SAM PETERSON, BEN HARRIS, JUSTIN ANDERSON, PARKER MAY
  METHODS INCLUDED IN THIS FILE
  setup()
  loop()
  captureData()
  bluetoothSend()
*/

#include <DFRobot_TFmini.h>
#include <Servo.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include <stdlib.h>
#include <SoftwareSerial.h>

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"


//New Servo and LiDAR distance objects
Servo servo;
SoftwareSerial mySerial = SoftwareSerial(5, 6); // RX, TX
DFRobot_TFmini  TFmini;
int distance, strength;

//Scan from 30 to 151 degrees
int pos = 30;

//Servo Control variables
int powerControl = 2;
int servoPin = 9;

//Array to store distance data in
int distanceData[122];

// Create the bluefruit object
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

//Begin initialization of the bluefruit
Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

int delayTime;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

//setup loop
void setup() {
  Serial.begin(115200);

  //set up pinmode for power control through transistor
  pinMode(2, OUTPUT);

  Serial.println(F("Starting up Bluetooth...."));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initializing the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in Command mode & check wiring?"));
  }
  Serial.println( F("Bluetooth Found!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info...");
  /* Print Bluefruit information */
  ble.info();

  //for debugging
  Serial.println(F("Using the Adafruit Bluefruit LE app to connect in UART mode"));

  ble.verbose(false);  // debug info is a little annoying after this point!

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  delayTime = 290;

  //set up the LiDAR sensor
  TFmini.begin(mySerial);

  //setup ble

  for (int i = 0; i <= 121; i++) {
    distanceData[i] = -1;
  }
}

//Do forever until power is cut
void loop() {
  // Check for incoming characters from Bluefruit every iteration if the ble is connected
  /* Wait for connection or capture data every 5 mins */
  bluefruitSS.listen();
  //while (!bluefruitSS.isListening()) {
  //Serial.println("Waiting for ble to listen");
  //}
  bluefruitSS.read();
  if (ble.isConnected()) {
    // Check for incoming characters from Bluefruit
    ble.println("AT+BLEUARTRX");
    ble.readline();
    if (strcmp(ble.buffer, "OK") == 0) {
      // no data
    }
    if (strcmp(ble.buffer, "START") == 0) {
      ble.print("AT+BLEUARTTX=");
      ble.println("ok");
      Serial.println("Sending Data!");
      bluetoothSend();
      ble.println("AT+BLEUARTRX");
      ble.readline();

    }
    else if (strcmp(ble.buffer, "DATA") == 0) {
      ble.print("AT+BLEUARTTX=");
      ble.println("ok");
      Serial.println("Collecting Data!");
      captureData();
      ble.println("AT+BLEUARTRX");
      ble.readline();
    }
  }


}

//------------------------------------------------------------------------------------------------------------//
//captureData() method
//INPUTS: NONE
//OUTPUTS: NONE
void captureData() {//loop through each angle and take measurements at each angle


  // turn on servo power
  digitalWrite(powerControl, HIGH);
  Serial.println("Servo Power is ON");
  for (pos = 30; pos <= 151; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.attach(servoPin);

    servo.write(pos);              // tell servo to go to position in variable 'pos'
    //delay(10);

    mySerial.listen();
    while (!mySerial.isListening()) {
      Serial.println("Waiting for TFmini to listen");
    }
    mySerial.read();
    // waits 15ms for the servo to reach the position
    delay(15);
    if (TFmini.measure()) {                    //Measure Distance and get signal strength
      distance = TFmini.getDistance();       //Get distance data
      strength = TFmini.getStrength();       //Get signal strength data

      //store distance data in array
      distanceData[pos - 30] = distance;

      //For Debugging/console printing
      Serial.print("Angle: ");
      Serial.println(pos);
      Serial.print("Distance = ");
      Serial.print(distance);
      Serial.println("cm");
      Serial.print("Strength = ");
      Serial.println(strength);
    }
    //delay(10);
  }
  delay(500);
  for (pos = 151; pos >= 30; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(20);                       // waits 15ms for the servo to reach the position
  }
  //turn off servo power
  digitalWrite(powerControl, LOW);
  Serial.println("Servo power is OFF");
  servo.detach();
}

//------------------------------------------------------------------------------------------------------------//


//------------------------------------------------------------------------------------------------------------//
//bluetoothSend() method
//INPUTS: None
//OUTPUTS: None
// Send data to Bluefruit
void bluetoothSend() {
  //string to send the data out on
  String compressedData = "";

  //concat into proper form and send out
  for (int i = 0; i < 30; i++) {
    compressedData.concat(i + 30);
    compressedData.concat(",");
    compressedData.concat(distanceData[i]);
    compressedData.concat(";");
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData);
  ble.println("");
  delay(50);
  compressedData = "";

  for (int i = 30; i < 60; i++) {
    compressedData.concat(i + 30);
    compressedData.concat(",");
    compressedData.concat(distanceData[i]);
    compressedData.concat(";");
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData);
  ble.println("");
  delay(50);
  compressedData = "";

  for (int i = 60; i < 90; i++) {
    compressedData.concat(i + 30);
    compressedData.concat(",");
    compressedData.concat(distanceData[i]);
    compressedData.concat(";");
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData);
  ble.println("");
  delay(50);
  compressedData = "";

  for (int i = 90; i < 100; i++) {
    compressedData.concat(i + 30);
    compressedData.concat(",");
    compressedData.concat(distanceData[i]);
    compressedData.concat(";");
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData);
  ble.println("");
  delay(50);
  compressedData = "";

  for (int i = 100; i < 110; i++) {
    compressedData.concat(i + 30);
    compressedData.concat(",");
    compressedData.concat(distanceData[i]);
    compressedData.concat(";");
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData);
  ble.println("");
  delay(50);
  compressedData = "";

  delay(50);
  for (int i = 110; i < 121; i++) {
    compressedData.concat(i + 30);
    compressedData.concat(",");
    compressedData.concat(distanceData[i]);
    compressedData.concat(";");
  }
  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData);
  ble.println("");
  delay(50);

  ble.println("AT+BLEUARTRX");
  ble.readline();

  // for debugging
  // check response stastus
  //if (! ble.waitForOK() ) {
  //Serial.println(F("Failed to send!t"));
  //}
}

//------------------------------------------------------------------------------------------------------------//
