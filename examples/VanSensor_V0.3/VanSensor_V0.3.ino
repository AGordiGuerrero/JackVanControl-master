/*
  VanSensor_V0.3.ino - TEst of sensor reading and approximation to the whole control 
                       Include the sending to Nextion HMI display using the JackVan library
  Created by A.Gordillo-Guerrero, May, 2021.
  Released into the public domain.
*/
#include <JackVanControl.h>

#define useNEXTION // for using Nextion HMI tactile screen

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


///////////////////////////////
// Variable definitions
//////////////////////////////

// timing variables
int currentMillis;
int previousMillisSensorRead;
int readSensorInterval=5000; //in milliseconds
int previousMillisCapRead;
int readCapInterval=500; //in milliseconds
/*
int controlInterval=1000; //in milliseconds
int dimInterval=10000; //in milliseconds
int delayRelay=1000; // in ms
int previousMillisControl;
int previousMillisPush; // last time a button was pushed
int previousMillisRearUpRelay;
int previousMillisRearDownRelay;
int previousMillisFrontUpRelay;
int previousMillisFrontDownRelay;
*/



// Define JackVan object
JackVan Van1(buttonPin, switchPin, VbatPin, CompPin,rearPressurePin, frontPressurePin, rearUpRelayPin, rearDownRelayPin, frontUpRelayPin, frontDownRelayPin);

int Nsamples=60;



void setup()
{
  Van1.init();

}

void loop()
{


//////////////////////////////////////////////////////////////////////
///////////////// CAPACITIVE BUTTON READING SECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////
  // Reading variables from sensors and updating Nextion Display
  currentMillis = millis();  
  if (currentMillis - previousMillisCapRead >= readCapInterval){
    previousMillisCapRead = currentMillis;
    Van1.readTouchbutton();
   }




//////////////////////////////////////////////////////////////////////
///////////////// SENSOR READING SECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////
  // Reading variables from sensors and updating Nextion Display
  currentMillis = millis();  
  if (currentMillis - previousMillisSensorRead >= readSensorInterval){
    previousMillisSensorRead = currentMillis;
  Van1.readandcheckAll_Nsamples(Nsamples);
  
  Van1.getStatus();
 
#ifdef useNEXTION
  Van1.updateNextionValues();
#endif
 
   }

}