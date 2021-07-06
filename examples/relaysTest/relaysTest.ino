
/*
  RelaysTest.ino - Simple test for Jack's van automatic control
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


void setup()
{
}

void loop()
{

  //Activate rearUpRelayPin
  Van1.turnON(rearUpRelayPin);
  delay(40);
  Van1.turnOFF(rearUpRelayPin);
  delay(40);

  //Activate rearDownRelayPin
  Van1.turnON(rearDownRelayPin);
  delay(40);
  Van1.turnOFF(rearDownRelayPin);
  delay(40);


  //Activate frontUpRelayPin
  Van1.turnON(frontUpRelayPin);
  delay(40);
  Van1.turnOFF(frontUpRelayPin);
  delay(40);

  //Activate frontDownRelayPin
  Van1.turnON(frontDownRelayPin);
  delay(40);
  Van1.turnOFF(frontDownRelayPin);
  delay(40);


}