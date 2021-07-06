/*
  NextionTest1.ino - Simple test to send dato to Nextion HMI Displayfor Jack's van automatic control
  Created by A.Gordillo-Guerrero, May, 2021.
  Released into the public domain.
*/
#include <JackVanControl.h>

#define useNEXTION // for using Nextion HMI tactile screen

#ifdef useNEXTION
#include "Nextion.h"
#include "HardwareSerial.h"
#endif

///////////////////////////////
// Pin definitions
//////////////////////////////
// For Nextion display we use the Serial2 ports on ESP32: RX2=16, TX2=17
// For IMU MPU6050, and LIDAR TOF10120 we use I2C protocol on ESP32 pins SDA=21, SCL=22
// For analog control we use a capacitive interrupt touch button, which is connected to GPIO27 (TouchPin 7)
uint8_t buttonPin = 27; // capacitive button pin
uint8_t switchPin = 26; // analog switch pin
uint8_t VbatPin   = 34; // pot pins
uint8_t CompPin   = 33; // pot pins
uint8_t rearPressurePin  = 32; // pot pins
uint8_t frontPressurePin = 35; // pot pins
uint8_t rearUpRelayPin     = 19;
uint8_t rearDownRelayPin   = 23;
uint8_t frontUpRelayPin    = 4;
uint8_t frontDownRelayPin  = 18;


// Define JackVan object
JackVan Van1(buttonPin, switchPin, VbatPin, CompPin,rearPressurePin, frontPressurePin, rearUpRelayPin, rearDownRelayPin, frontUpRelayPin, frontDownRelayPin);

int Nsamples=60;



#ifdef useNEXTION

char buff[50];

//HardwareSerial nextion(17, 16);// Nextion TX to pin 17 and RX to pin 16 of ESP32 on Serial2
//Nextion myNextion(nextion, 9600); //create a Nextion object named myNextion using the nextion serial port @ 9600bps

// Declare your Nextion objects - Example (page id = 0, component id = 1, component name = "b0") 
// Page 0 (Main)
NexText texvalVbat   = NexText(0, 25, "texvalVbat"); 
NexText texvalRear0  = NexText(0, 15, "texvalRear"); 
NexText texvalFront0 = NexText(0, 16, "texvalFront"); 
NexText texvalComp0  = NexText(0, 17, "texvalComp"); 
NexText texvalAngle0 = NexText(0, 18, "texvalAngle"); 
NexText texMode0     = NexText(0, 10, "texMode"); 
NexText texMessage0  = NexText(0, 12, "texMessage"); 
NexText texHRear0    = NexText(0, 13, "texHRear"); 
NexText texHFront0   = NexText(0, 14, "texHFront"); 
NexButton butPark0    = NexButton(0, 4, "butPark");
NexButton butRoad0    = NexButton(0, 5, "butRoad");
NexButton butTrail0   = NexButton(0, 6, "butTrail");
NexButton butManual0  = NexButton(0, 7, "butManual");
NexButton butAuto0    = NexButton(0, 8, "butAuto");
NexButton butUpFront0  = NexButton(0, 21, "butUpFront");
NexButton butDownFront0= NexButton(0, 22, "butDownFront");
NexButton butUpRear0   = NexButton(0, 19, "butUpRear");
NexButton butDownRear0 = NexButton(0, 20, "butDownRear");

/*
// Register a button object to the touch event list.  
NexTouch *nex_listen_list[] = {
  &butPark0,
  &butRoad0, 
  &butTrail0,
  &butManual0,
  &butAuto0,
  &butUpFront0,
  &butDownFront0,
  &butUpRear0,
  &butDownRear0,
  NULL
};
*/

#endif



void setup()
{
  Van1.init();
#ifdef useNEXTION

  Serial.begin(9600); // Serial comunication with Nextion display for debug

  Serial2.begin(9600); // Serial comunication with Nextion display using serial2 port
  // Nextion definitions
  nexInit();

  sendCommand("vis texMessage,0"); // Hide texMessage at startup

/*
  // Register the Push event callback function of the components
  butPark0.attachPush(butPark0Callback, &butPark0);
  butRoad0.attachPush(butRoad0Callback, &butRoad0);
  butTrail0.attachPush(butTrail0Callback, &butTrail0);
  butAuto0.attachPush(butAuto0Callback, &butAuto0);
  butManual0.attachPush(butManual0Callback, &butManual0);
  butUpFront0.attachPush(butUpFront0Callback, &butUpFront0);
  butDownFront0.attachPush(butDownFront0Callback, &butDownFront0);
  butUpRear0.attachPush(butUpRear0Callback, &butUpRear0);
  butDownRear0.attachPush(butDownRear0Callback, &butDownRear0);
*/

#endif






}

void loop()
{

  Van1.readandcheckAll_Nsamples(Nsamples);
  
  Van1.getStatus();

#ifdef useNEXTION
  // Updating Nextion screen text fields
    sprintf(buff, "%.1fV", Van1.Vbat); // displaying Vbat
    texvalVbat.setText(buff);
    utoa(Van1.PRear, buff, 10); // unsigned integer to char
    texvalRear0.setText(buff);
    utoa(Van1.PFront, buff, 10);// unsigned integer to char
    texvalFront0.setText(buff);
    utoa(Van1.PComp, buff, 10);// unsigned integer to char
    texvalComp0.setText(buff);
    sprintf(buff, "%d\xB0", Van1.angle); // displaying º symbol
    texvalAngle0.setText(buff);
     //creating strings for height text fields
    sprintf(buff, "%d / %d cm", Van1.HRear, Van1.desHRear);
    texHRear0.setText(buff);
    sprintf(buff, "%d / %d cm", Van1.HFront, Van1.desHFront);
    texHFront0.setText(buff);
#endif



  delay(3000);

}