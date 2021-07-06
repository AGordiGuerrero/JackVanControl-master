/*
  ReadPressuresNsamples.ino - Simple test for Jack's van automatic control
  
	Read Nsamples of three analog pressure sensors averaging.

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

int Nsamples  = 40;

// Define JackVan object
JackVan Van1(buttonPin, switchPin, VbatPin, CompPin,rearPressurePin, frontPressurePin, rearUpRelayPin, rearDownRelayPin, frontUpRelayPin, frontDownRelayPin);


void setup()
{
  Van1.init();

}

void loop()
{

  Van1.read_pressures_Nsamples(Nsamples);

  Serial.print("  Readed pressures at main source (PSI): ");
  Serial.print("Pressure Front: ");
  Serial.println(Van1.PFront);
  Serial.print("Pressure Rear: ");
  Serial.println(Van1.PRear);
  Serial.print("Pressure Comp: ");
  Serial.println(Van1.PComp);
  delay(3000);

}