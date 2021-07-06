//
// VanControl_V0.52.ino
//
// VOLKSWAGEN VAN CONTROL FOR JACOBO MONTAÑO FERNÁNDEZ (JackVanControl)
//---------------------******--------------------------//
//
// Include:
//    - DO INCLUDE Communication with Nextion GUI using serial comunication (to be used with JackVan_V9.hmi) using #define useNEXTION
//         - Use only one page for all controls.
//         - Include dimming of the screen after 10 seconds without use.
//         - Hide buttons of Manual Mode in all other modes.g
//         - Include push button callbacks in secondary function file (functions.ino)
//    - Read analog values from simulation potentiometers
//    - Include accelerometer MCU6050 meassure to obtain th inclination angle.
//    - Include three pressure sensors (one for compressor out, one for rear suspension, one for front suspension)
//    - Include four relays board to control electrovalves (WITH NEGATIVE LOGIC).
//    - Include two TOF10120 LIDAR distance sensor de measure height
//    - Switch the LEDs without delay
//    - Include lecture and printing on screen of van's battery voltage.
//    - Include analog 1P4T with capacitive button to adjust the mode without the LCD screen.
//    - Definition of the button action as simple digital read calling a standard function.
//    - Included control:
//         - Perform pressure corrections only if a MODE button was pushed
//         - Perform angle corrections only if MODE AUTO button was pushed
//         - Increase height only if((!Van1.flagCompDeflated)&&(Van1.flagCompInflated))
//         - First try to deflate, next try to inflate.
//    - Change definitions to use object oriented programing using "JackVan-Master" library           
//    - Perform checks after measurements to mark with flags if they are out of the sensible range
//    - Include NtrialsFront and NtrialsRear, and only perform NtrialsMax control trials.
//    - Include screen dimming routine  
//  
//  
//  
// Thanks to:
//    - www.randomnerdtutorials.com
//    - Arduino online open community. Thank you all!
// 
// Licensed under Creative Commons (SH-BY-NC). @AGordiGuerrero
//.
// Antonio Gordillo Guerrero
// anto@unex.es
// @AGordiGuerrero
// Smart Open Lab. Escuela Politécnica de Cáceres. Universidad de Extremadura.
// www.smartopenlab.com
// @SolEpcc
//
//--------------------******---------------------------//

#define SerialDEBUG // for debuging via serial port

#define useNEXTION // for using Nextion HMI tactile screen


#include <JackVanControl.h>


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

//uint8_t ledPin = 25; // optional LED pin



///////////////////////////////
// Variable definitions
//////////////////////////////

// timing variables
int currentMillis;
int previousMillisSensorRead;
int readSensorInterval=5000; //in milliseconds
int previousMillisCapRead;
int readCapInterval=500; //in milliseconds
int controlInterval=1000; //in milliseconds
int dimInterval=10000; //in milliseconds
int delayRelay=1000; // in ms
int previousMillisControl;
//int previousMillisPush; // last time a button was pushed
int previousMillisRearUpRelay;
int previousMillisRearDownRelay;
int previousMillisFrontUpRelay;
int previousMillisFrontDownRelay;
#ifdef SerialDEBUG
int previousMillisSerialPrint;
int printSerialInterval=5000; //in milliseconds
#endif



// Control variables
//int tolP=10; //tolerance (in PSI) for the desired pressure
int tolH=1; //tolerance (in cm) for the desired height
int tolAngle=1; //tolerance (in degrees) for the desired angle

//int lastsimreadPRear=14; //USED FOR SIMULATION
char buff[50];

  // Define JackVan object
JackVan Van1(buttonPin, switchPin, VbatPin, CompPin, rearPressurePin, frontPressurePin, rearUpRelayPin, rearDownRelayPin, frontUpRelayPin, frontDownRelayPin);


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


/////////// SETUP ////////////
void setup() {

#ifdef SerialDEBUG
  //Serial.begin(9600); //Start Serial monitor in 9600. Take care if it is already defined at library!
#endif

  Van1.init();


#ifdef useNEXTION

  // Nextion definitions
  nexInit();

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
#endif

}


/////////// LOOP ////////////
void loop() {



#ifdef useNEXTION

//////////////////////////////////////////////////////////////////////
///////////////// NEXTION EVENT DETECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////

  // When a Push or push event occured every time, the corresponding component[right page id and component id] in touch event list will be asked.
  nexLoop(nex_listen_list); 


//////////////////////////////////////////////////////////////////////
///////////////// SCREEN DIMMING SECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////
  //decreasing dim level of the screen after a while, until any button is pushed
  currentMillis = millis();  
  if ((currentMillis - Van1.previousMillisPush >= dimInterval)&&(!Van1.flagDim)){
  //start dimming loop
      uint8_t i=0;
      for (i=0; i<=200; i++){ // 200 steps from 100 till 5
          sprintf(buff, "dim=%d", 100-(100-5)*i/200);
          sendCommand(buff);
          delay(40);
          Van1.flagDim=1;
      }
   }
  

#endif


//////////////////////////////////////////////////////////////////////
///////////////// CAPACITIVE BUTTON READING SECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////
  // Reading variables from sensors and updating Nextion Display
  currentMillis = millis();  
  if (currentMillis - previousMillisCapRead >= readCapInterval){
    previousMillisCapRead = currentMillis;
    Van1.readTouchbutton();
   }


////////////////////////////////////////////////////////////////////////////
///////////////// Checking relays millis ////////////////////
////////////////////////////////////////////////////////////////////////////
checkAllRelaysMillis(); // to close relays without additional timing


//////////////////////////////////////////////////////////////////////
///////////////// SENSOR READING SECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////
  // Reading variables from sensors and updating Nextion Display
  currentMillis = millis();  
  if (currentMillis - previousMillisSensorRead >= readSensorInterval){
    previousMillisSensorRead = currentMillis;

    Van1.readandcheckAll_Nsamples(60);
  
    Van1.getStatus();
 
    #ifdef useNEXTION
    Van1.updateNextionValues();
    #endif

   }



//////////////////////////////////////////////////////////////////////
///////////////// CONTROL SECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// controlling something if needed
//////////////////////////////////////////////////////////////////////

// Only controlling each controlInterval, in milliseconds
 
currentMillis = millis();  
if (currentMillis - previousMillisControl >= controlInterval){

    previousMillisControl = currentMillis;

    // Trying to control using the height sensors    
    if( (Van1.letsControlHeight) && Van1.flagHeights ){ // only control if heights are in range
//    if(0){
  
      // Reading sensors before control
      Van1.readHeights_Nsamples_discard(60);
      Van1.checkHeights();

      Van1.readPressures_Nsamples(20);
      Van1.checkPComp();

      //////////////////////////////////////////////////////////
      // Controlling Rear Height
      //////////////////////////////////////////////////////////
      if(abs(Van1.desHRear-Van1.HRear)>tolH){ //we have to change Rear Pressure

         Van1.flagCorrectRear=0; // flag 0 when height is not correct
         Van1.NTrialsRear++; // Increase the number of trials
    
         tryControlRearHeight();

      } // end if rear height is outside tolerance
      else {  // if rear height is ok
        digitalWrite(rearUpRelayPin, HIGH);
        digitalWrite(rearDownRelayPin, HIGH);
        Van1.flagCorrectRear=1; // Rear Height is correct!
        #ifdef SerialDEBUG
        Serial.print("REAR HEIGHT IS CORRECT AFTER");
        Serial.print(Van1.NTrialsRear);
        Serial.println(" TRIALS");
        #endif
        #ifdef useNEXTION
        sendCommand("texHRear.pco=BLACK");
        #endif
      }


      //////////////////////////////////////////////////////////
      // Controlling Front Height      
      //////////////////////////////////////////////////////////
      if(abs(Van1.desHFront-Van1.HFront)>tolH){ //we have to change Front Pressure

         Van1.flagCorrectFront=0; // flag 0 when height is not correct
         Van1.NTrialsFront++; // Increase the number of trials
    
        tryControlFrontHeight();

         
      } // end if Van1.desHFront out of tolerance)
      else {  
         digitalWrite(frontUpRelayPin, HIGH); // turn it OFF
         digitalWrite(frontDownRelayPin, HIGH); // turn it OFF
         Van1.flagCorrectFront=1; // Front height is correct!
         #ifdef SerialDEBUG
         Serial.print("FRONT HEIGHT IS CORRECT AFTER ");
         Serial.print(Van1.NTrialsFront);
         Serial.println(" TRIALS");
         #endif
         #ifdef useNEXTION
         sendCommand("texHFront.pco=BLACK");
         #endif
     }
  
    if( Van1.flagCorrectFront && Van1.flagCorrectRear ){
        
        Van1.letsControlHeight=0; // everything is correct
        // Put to zero the number of trials for the next control action
        Van1.NTrialsFront=0; 
        Van1.NTrialsRear=0; 
#ifdef useNEXTION
        sendCommand("vis texMessage,0"); // Show texMessage
        sendCommand("texHRear.pco=BLACK");
        sendCommand("texHFront.pco=BLACK");
#endif
    }

    // Reading sensors after control
//    Van1.readHeights_Nsamples_discard(40);
//    Van1.readAngle_Nsamples(30);
//    Van1.readPressures_Nsamples(30);

#ifdef useNEXTION
  // refresh values after control
  Van1.updateNextionValues();
#endif
     
  } // end if Van1.letsControlHeight && flagHeights 
  else if (!Van1.letsControlAngle) {
    Van1.turnAllRelaysOFF();
  }


 
//////////////////////////////////////////////////////////////////////
// controlling angle if needed
//////////////////////////////////////////////////////////////////////
//    if(0){
  if(Van1.letsControlAngle && Van1.flagAngle){ // only control if measured angle is in correct range

       #ifdef SerialDEBUG
         Serial.println("Trying to control angle...");
       #endif

      // Reading sensors before control
      Van1.readAngle_Nsamples(40);
      Van1.checkAngle();
      Van1.readPressures_Nsamples(30);
      Van1.checkPComp();

      if (Van1.flagDeflated==0){
   
         tryDeflateBoth();

      }// end if Van1.flagDeflated==0


     if (Van1.flagDeflated==1){ // now we can try to reach horizontality
        
        tryControlAngle();
    
     } // end if Deflated=1

    if(abs(Van1.angle)<tolAngle){
        Van1.letsControlAngle=0; // everything is correct!
    }

/*
    // Reading sensors after control
//    readheigths_Nsamples(30);
    Van1.readAngle_Nsamples(40);
    Van1.checkAngle();
    Van1.readPressures_Nsamples(30);
    Van1.checkPComp();
*/

#ifdef useNEXTION
  // refresh values after control
  Van1.updateNextionValues();
#endif
       
  } // end if Van1.letsControlAngle
  else if (!Van1.letsControlHeight) {
    Van1.turnAllRelaysOFF();
  }


} // end if millis control


/*  
////////////////////////////////////////////////////////////////////////////
///////////////// SERIAL COMMUNICATION SECTION /////////////////////////////
////////////////////////////////////////////////////////////////////////////
#ifdef SerialDEBUG
  // Sending data via serial port for debug
  currentMillis = millis();  
  if (currentMillis - previousMillisSerialPrint >= printSerialInterval){
    
    previousMillisSerialPrint = currentMillis;
    Van1.getStatus();

   }
#endif
*/

  //delay(20); // bit of delay...
  
}  //end main loop