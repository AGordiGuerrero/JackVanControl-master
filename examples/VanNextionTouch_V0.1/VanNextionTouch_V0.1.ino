/*
  VanNextionTouch_V0.1.ino - TEst of sensor reading and approximation to the whole control 
                       Include the sending to Nextion HMI display using the JackVan library
                       Include reading touch events from the Nextion display
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

int delayRelay=1000; // in ms
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

// Register a button object to the touch event list.  
NexTouch *nex_listen_list[] = {
  &Van1.butPark0,
  &Van1.butRoad0, 
  &Van1.butTrail0,
  &Van1.butManual0,
  &Van1.butAuto0,
  &Van1.butUpFront0,
  &Van1.butDownFront0,
  &Van1.butUpRear0,
  &Van1.butDownRear0,
  NULL
};


void setup()
{
  Van1.init();

  sendCommand("vis texMessage,0"); // Hide texMessage at startup

  // Register the Push event callback function of the components
  Van1.butPark0.attachPush(butPark0Callback, &Van1.butPark0);
  Van1.butRoad0.attachPush(butRoad0Callback, &Van1.butRoad0);
  Van1.butTrail0.attachPush(butTrail0Callback, &Van1.butTrail0);
  Van1.butAuto0.attachPush(butAuto0Callback, &Van1.butAuto0);
  Van1.butManual0.attachPush(butManual0Callback, &Van1.butManual0);
  Van1.butUpFront0.attachPush(butUpFront0Callback, &Van1.butUpFront0);
  Van1.butDownFront0.attachPush(butDownFront0Callback, &Van1.butDownFront0);
  Van1.butUpRear0.attachPush(butUpRear0Callback, &Van1.butUpRear0);
  Van1.butDownRear0.attachPush(butDownRear0Callback, &Van1.butDownRear0);



}

void loop()
{

   nexLoop(nex_listen_list); 


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



/// Button callbacks functions
// Button butPark0 component Push callback function. 
// When the Parking button is released, the text on the texMode changes and global variables set. 
void butPark0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=1; // we have to control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=1; // setting mode
  Van1.texMode0.setText("PARKING MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Parking Mode ON");
#endif
  Van1.desHFront=Van1.desHParkFront;
  Van1.desHRear=Van1.desHParkRear;
}

void butRoad0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=1; // we have to control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=2; // setting mode
  Van1.texMode0.setText("ROAD MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Road Mode ON");
#endif
  Van1.desHFront=Van1.desHRoadFront;
  Van1.desHRear=Van1.desHRoadRear;
}


void butAuto0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=0; // we do not control height
  Van1.letsControlAngle=1; // we do control angle
  Van1.flagDeflated=0; // to start deflating
  Van1.mode=0; // setting mode
  Van1.texMode0.setText("AUTOLEVEL MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Auto Mode ON");
#endif
// in auto mode we choose the parameters of the Parking mode
  Van1.desHFront=Van1.desHParkFront;
  Van1.desHRear=Van1.desHParkRear;
}


void butTrail0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=1; // we have to control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=3; // setting mode
  Van1.texMode0.setText("TRAIL MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Trail Mode ON");
#endif 
  Van1.desHFront=Van1.desHTrailFront;
  Van1.desHRear=Van1.desHTrailRear;
}


void butManual0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=0; // we do not control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=4; // setting mode
  Van1.texMode0.setText("MANUAL MODE");
  // Showing manual control buttons
  sendCommand("vis butUpFront,1"); // Show button
  sendCommand("vis butDownFront,1"); // Show button
  sendCommand("vis butUpRear,1"); // Show button
  sendCommand("vis butDownRear,1"); // Show button

#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Manual Mode ON");
#endif
  Van1.desHFront=200;
  Van1.desHRear=200;

}

void butUpFront0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("butUpFront pushed");
#endif

  // here we have to change heigth by activating a relay
//  Serial.println("butUpFront Pushed");
   digitalWrite(frontUpRelayPin, LOW);
   delay(delayRelay);
   digitalWrite(frontUpRelayPin, HIGH);

}

void butDownFront0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("butDownFront pushed");
#endif

  // here we have to change heigth by activating a relay
//  Serial.println("butDownFront Pushed");
   digitalWrite(frontDownRelayPin, LOW);
   delay(delayRelay);
   digitalWrite(frontDownRelayPin, HIGH);


}


void butUpRear0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("butUpRear pushed");
#endif

  // here we have to change heigth by activating a relay
  //Serial.println("butUpRear Pushed");

   digitalWrite(rearUpRelayPin, LOW);
   delay(delayRelay);
   digitalWrite(rearUpRelayPin, HIGH);

}

void butDownRear0Callback(void *ptr) {
  // saving push time
  //previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("butDownRear pushed");
#endif

  // here we have to change heigth by activating a relay
  //Serial.println("butDownRear Pushed");

   digitalWrite(rearDownRelayPin, LOW);
   delay(delayRelay);
   digitalWrite(rearDownRelayPin, HIGH);

}
