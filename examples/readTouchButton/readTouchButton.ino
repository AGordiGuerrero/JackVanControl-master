/*
  readTouchbutton.ino - Read of capacitive touch button for Jack's van automatic control
  Created by A.Gordillo-Guerrero, May, 2021.
  Released into the public domain.
*/
#include <JackVanControl.h>

// Pin definitions
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

// timing variables
int currentMillis;
int previousMillisCapButtonRead;
int CapButtonReadInterval=500; //in milliseconds
int previousMillisStatusRead;
int StatusReadInterval=2000; //in milliseconds


void setup()
{
  Van1.init();
}

void loop()
{

 // Reading variables from sensors and updating Nextion Display
  currentMillis = millis();  
  if (currentMillis - previousMillisCapButtonRead >= CapButtonReadInterval){
    previousMillisCapButtonRead = currentMillis;
    Van1.readTouchbutton();
  }
  
  if (currentMillis - previousMillisStatusRead >= StatusReadInterval){
    previousMillisStatusRead = currentMillis;
  Van1.getStatus();
  }

}