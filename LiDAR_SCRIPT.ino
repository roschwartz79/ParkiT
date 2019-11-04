/*
  ParkIT peripheral sensor script - ECE SENIOR DESIGN PROJECT
  DESIGNED BY: ROB SCHWARTZ, SAM PETERSON, BEN HARRIS, JUSTIN ANDERSON, PARKER MAY

  METHODS INCLUDED IN THIS FILE
  setup()
  loop()
  captureData()

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
SoftwareSerial mySerial = SoftwareSerial(12, 13); // RX, TX
DFRobot_TFmini  TFmini;
int distance, strength;

//Scan from 30 to 151 degrees
int pos = 30;

int distanceData[121];

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

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
  //bluefruitSS.begin(115200);

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

  Serial.println(F("Using the Adafruit Bluefruit LE app to connect in UART mode"));

  ble.verbose(false);  // debug info is a little annoying after this point!

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  //servo control on pin 8
  servo.attach(8);
  //output the angle to control the servo at on startup
  servo.writeMicroseconds(1500);
  delay(100);

  delayTime = 0;

  //set up the LiDAR sensor
  //TFmini.begin(mySerial);

  //setup ble 

  for (int i = 0; i < 121; i++) {
    distanceData[i] = i;
  }
}

//Do forever <3
void loop() {
  // Check for incoming characters from Bluefruit every iteration if the ble is connected
  /* Wait for connection or capture data every 5 mins */
  bluefruitSS.listen();
  while (!bluefruitSS.isListening()){
    Serial.println("Waiting for ble to listen");
  }
  if (ble.isConnected()) {
    // Check for incoming characters from Bluefruit
    ble.println("AT+BLEUARTRX");
    ble.readline();
    if (strcmp(ble.buffer, "OK") == 0) {
      // no data
    }
    else if (strcmp(ble.buffer, "START") == 0) {
      Serial.println("Sending Data!");
      bluetoothSend();
    }
  }

  //Do we want to capture new data
  if (delayTime  == 300) {
    //capture the new data
    captureData();

    //delay 4.95 minutes -> reset the servo angle to 30 degrees, start the program up again after 5 minutes
    delayTime = 0;
    servo.write(30);
  }

  //if we aren't capturing new data, increase delay
  delayTime += 1;
  delay(1000);
  Serial.println(delayTime);
}

//------------------------------------------------------------------------------------------------------------//
//captureData() method
//INPUTS: NONE
//OUTPUTS: NONE
void captureData() {
  mySerial.listen();
  while (!mySerial.isListening()){
    Serial.println("Waiting for TFmini to listen");
  }
  //loop through each angle and take measurements at each angle
  for (pos = 30; pos <= 151; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    Serial.println("Rotating...");
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
    delay(1000);
  }

  for (pos = 151; pos >= 30; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
  }
}

//------------------------------------------------------------------------------------------------------------//


//------------------------------------------------------------------------------------------------------------//
//bluetoothSend(int) method
//INPUTS: The distance measurement
//OUTPUTS: None
// Send data to Bluefruit
void bluetoothSend() {
  String compressedData1 = "";
  String compressedData2 = "";
  String compressedData3 = "";
  String compressedData4 = "";
  String compressedData5 = "";



  for (int i = 0; i < 30; i++) {
    compressedData1.concat(i + 30);
    compressedData1.concat(",");
    compressedData1.concat(distanceData[i]);
    compressedData1.concat(";");
  }

  for (int i = 30; i < 60; i++) {
    compressedData2.concat(i + 30);
    compressedData2.concat(",");
    compressedData2.concat(distanceData[i]);
    compressedData2.concat(";");
  }

for (int i = 60; i < 90; i++) {
    compressedData3.concat(i + 30);
    compressedData3.concat(",");
    compressedData3.concat(distanceData[i]);
    compressedData3.concat(";");
  }
for (int i = 90; i < 115; i++) {
    compressedData4.concat(i + 30);
    compressedData4.concat(",");
    compressedData4.concat(distanceData[i]);
    compressedData4.concat(";");
  }    
for (int i = 115; i < 121; i++) {
    compressedData5.concat(i + 30);
    compressedData5.concat(",");
    compressedData5.concat(distanceData[i]);
    compressedData5.concat(";");
  }  

  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData1);
  ble.println("");
  delay(100);
  ble.print("AT+BLEUARTTX=");
  ble.println(compressedData2);
  ble.println("");
  delay(100);
  ble.print("AT+BLEUARTTX=");  
  ble.println(compressedData3);
  ble.println("");
  delay(100);
  ble.print("AT+BLEUARTTX=");  
  ble.println(compressedData4);
  ble.println("");
  delay(100);
  ble.print("AT+BLEUARTTX=");  
  ble.println(compressedData5);
  ble.println("");
  delay(100);
  ble.print("AT+BLEUARTTX=");  
  ble.println("144");

  // check response stastus
  if (! ble.waitForOK() ) {
    Serial.println(F("Failed to send!t"));
  }
}

//------------------------------------------------------------------------------------------------------------//


//------------------------------------------------------------------------------------------------------------//
//bluetoothReceive() method
//INPUTS:
//OUTPUTS:

//------------------------------------------------------------------------------------------------------------//
