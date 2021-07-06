//
// VanControl_V0.32.ino
//
// VOLKSWAGEN VAN CONTROL FOR JACOBO MONTAÑO FERNÁNDEZ (JackVanControl)
//---------------------******--------------------------//
//
// Include:
//    - DO INCLUDE Communication with Nextion GUI using serial comunication (to be used with JackVan_V9.hmi) using #define useNEXTION
//         - Use only one page for all controls.
//         - Include dimming of the screen after 10 seconds without use.
//         - Hide buttons of Manual Mode in all other modes.g
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
//         - Increase height only if CompRead>CompDeflatedThreshold
//    - Change definitions to use object oriented programing using "JackVan-Master" library           
//    - Perform checks after measurements to mark with flags if they are out of the sensible range//
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

//#define useNEXTION // for using Nextion HMI tactile screen


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
int previousMillisPush; // last time a button was pushed
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
int CapThresh=30; // Threshold for capacitive button
int CompDeflatedThresh=90; // Threshold for compressor pressure
int PLowerThresh=90; // Threshold for deflation pressure
//int lastsimreadPRear=14; //USED FOR SIMULATION
char buff[50];

  // Define JackVan object
JackVan Van1(buttonPin, switchPin, VbatPin, CompPin, rearPressurePin, frontPressurePin, rearUpRelayPin, rearDownRelayPin, frontUpRelayPin, frontDownRelayPin);


#ifdef useNEXTION

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
#endif




/////////// SETUP ////////////
void setup() {

  Van1.init();

#ifdef SerialDEBUG
  Serial.begin(9600); //Start Serial monitor in 9600
  Serial.println("Welcome to Jack Van Control serial monitor\n Starting....");  
#endif

#ifdef useNEXTION

  Serial2.begin(9600); // Serial comunication with Nextion display using serial2 port
  // Nextion definitions
  nexInit();

  sendCommand("vis texMessage,0"); // Hide texMessage at startup

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

#endif

}


/////////// LOOP ////////////
void loop() {

//  Serial.println("Hellofromloop...");  


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
  if ((currentMillis - previousMillisPush >= dimInterval)&&(!Van1.flagDim)){
  //start dimming loop
      uint8_t i=0;
      for (i=0; i<=200; i++){ // 200 steps from 100 till 5
          sprintf(buff, "dim=%d", 100-(100-5)*i/200);
          sendCommand(buff);
          delay(20);
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



//////////////////////////////////////////////////////////////////////
///////////////// SENSOR READING SECTION /////////////////////////////
//////////////////////////////////////////////////////////////////////
  // Reading variables from sensors and updating Nextion Display
  currentMillis = millis();  
  if (currentMillis - previousMillisSensorRead >= readSensorInterval){
    previousMillisSensorRead = currentMillis;

    Van1.readVbat();
    Van1.readPressures_Nsamples(20);
    Van1.checkPComp();
    Van1.readHeights_Nsamples_discard(60);
    Van1.checkHeights();
    Van1.readAngle_Nsamples(60);   
    Van1.checkAngle();

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
    
    if( (Van1.letsControlHeight) && Van1.flagHeights ){ // only control if heights are in range
//    if(0){
  
      // Reading sensors before control
      Van1.readHeights_Nsamples_discard(60);
      Van1.checkHeights();

      Van1.readPressures_Nsamples(20);
      Van1.checkPComp();

      
      // Controlling Rear Height
      if(abs(Van1.desHRear-Van1.HRear)>tolH){ //we have to change Rear Pressure

         Van1.flagRear=0; // flag 0 when rear Height is not correct

         if(Van1.desHRear>Van1.HRear){//we have to increase Rear Height

              #ifdef SerialDEBUG
              Serial.println("Trying to inflate Rear UP!");
              #endif

//               if(Van1.PComp>CompDeflatedThresh){// then we can increase Pressure
  //              PRear++; // increase Rear Height for simulation

                // Take care of inverted logic with the relay module
                if(digitalRead(rearUpRelayPin)){ // if it is OFF, turn it ON it and save the change time
            
                 if((!Van1.flagCompDeflated)&&(Van1.flagCompInflated)){// then we can increase rear Pressure
                    previousMillisRearUpRelay=millis();      
//                  // Activate corresponding delay (turn input to LOW)
                    digitalWrite(rearUpRelayPin, LOW);
                    // Assure that the contrary is OFF (high)
                    digitalWrite(rearDownRelayPin, HIGH);
                    #ifdef SerialDEBUG
                      Serial.println("Lets inflate REAR UP!");
                    #endif
#ifdef useNEXTION
                    sendCommand("texHRear.pco=0x0408"); // Dark Green
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
                }
                else{// else if (!Van1.flagCompDeflated)&&(Van1.flagCompInflated) we cannot increase Pressure
                     digitalWrite(rearUpRelayPin, HIGH); // turn OFF
                #ifdef SerialDEBUG
                    Serial.println("Cannot Inflate REAR UP!");
                #endif
#ifdef useNEXTION
                sendCommand("vis texMessage,1"); // Show texMessage
                sendCommand("texHRear.pco=21087"); // Light Blue
                sendCommand("texMessage.txt=\"Low compressor pressure.\""); // Change text
#endif
                }// end if (!Van1.flagCompDeflated)&&(Van1.flagCompInflated)
            }
            else{   // else if the pin was LOW, turn it off (HIGH) after delayRelay ms  
                    if(millis() - previousMillisRearUpRelay > delayRelay){
                        digitalWrite(rearUpRelayPin, HIGH); // turn OFF
                        #ifdef SerialDEBUG
                          Serial.println("Stop Inflating REAR UP.");
                        #endif
                      }
                }
           
             
         } // end if Van1.desHRear>Van1.HRear

         else if(Van1.desHRear<Van1.HRear){//we have to decrease Rear Height
   //        PRear--; // decrease Rear Height for simulation
           // Activate corresponding delay
              #ifdef SerialDEBUG
              Serial.println("Trying to deflate Rear DOWN!");
              #endif

               if(digitalRead(rearDownRelayPin)){ // if it is HIGH (off), activate it and save the change time
                // Take care of inverted logic with the relay module
                    // Activate corresponding delay
                    digitalWrite(rearDownRelayPin, LOW);
                    previousMillisRearDownRelay=millis();
                    // Assure that the contrary is OFF (high)
                    digitalWrite(rearUpRelayPin, HIGH);
                    #ifdef SerialDEBUG
                      Serial.println("Lets deflate REAR DOWN!");
                    #endif
#ifdef useNEXTION
                    sendCommand("texHRear.pco=0xB000"); // Dark Red
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
               }    
                else{// else if the pin was LOW, turn it off (HIGH) after delayRelay ms    
                    if(millis() - previousMillisRearDownRelay > delayRelay){
                        digitalWrite(rearDownRelayPin, HIGH);
                    #ifdef SerialDEBUG
                      Serial.println("Stop deflating REAR DOWN");
                    #endif
                    }
                }
          }    
      }
      else {  // if rear heights is ok
              digitalWrite(rearUpRelayPin, HIGH);
              digitalWrite(rearDownRelayPin, HIGH);
              Van1.flagRear=1; // Rear Height is correct!
              #ifdef SerialDEBUG
                 Serial.println("REAR HEIGHT IS CORRECT");
              #endif
#ifdef useNEXTION
              sendCommand("texHRear.pco=BLACK");
#endif
      }
      
      // Controlling Front Pressure      
      if(abs(Van1.desHFront-Van1.HFront)>tolH){ //we have to change Front Pressure

         Van1.flagFront=0; // flag 0 when height is not correct

         if(Van1.desHFront>Van1.HFront){//we have to increase Front height

              #ifdef SerialDEBUG
              Serial.println("Trying to inflate Front UP!");
              #endif

                // Take care of inverted logic with the relay module
             if(digitalRead(frontUpRelayPin)){ // if it is OFF, turn it ON it and save the change time
            
//             if(Van1.PComp>CompDeflatedThresh){// then we can increase Pressure
                 if((!Van1.flagCompDeflated)&&(Van1.flagCompInflated)){// then we can increase rear Pressure
                      // Activate corresponding delay
                        // Take care of inverted logic with the relay module
                        // Activate corresponding delay
                          digitalWrite(frontUpRelayPin, LOW);
                          previousMillisFrontUpRelay=millis();
                          // Assure that the contrary is OFF (high)
                          digitalWrite(frontDownRelayPin, HIGH);
                          #ifdef SerialDEBUG
                            Serial.println("Lets inflate Front UP!");
                          #endif

#ifdef useNEXTION
                    sendCommand("texHFront.pco=0x0408"); // Dark Green
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
                 }
                  else{// else if (!Van1.flagCompDeflated)&&(Van1.flagCompInflated) we cannot increase Pressure
                     digitalWrite(frontUpRelayPin, HIGH); // turn OFF
                #ifdef SerialDEBUG
                    Serial.println("Cannot Inflate FRONT UP!");
                #endif
#ifdef useNEXTION
                sendCommand("vis texMessage,1"); // Show texMessage
                sendCommand("texHFront.pco=21087"); // Light Blue
                sendCommand("texMessage.txt=\"Low compressor pressure.\""); // Change text
#endif
                }// end if (!Van1.flagCompDeflated)&&(Van1.flagCompInflated)
             }
            else{// else if the pin was LOW, turn it off (HIGH) after delayRelay ms  
                    if(millis() - previousMillisFrontUpRelay > delayRelay){
                        digitalWrite(frontUpRelayPin, HIGH); // turn it OFF
                    #ifdef SerialDEBUG
                        Serial.println("Stop inflating Front.");
                    #endif
                    }
                }
         }
         else if(Van1.desHFront<Van1.HFront){//we have to decrease Front Pressure
    //            PFront--; // decrease front Pressure for simulation

              #ifdef SerialDEBUG
              Serial.println("Trying to deflate Front DOWN!");
              #endif

                // Activate corresponding delay if it is not already activated         
              
               if(digitalRead(frontDownRelayPin)){ // if it is HIGH (off), activate it and save the change time
                // Take care of inverted logic with the relay module
                // Activate corresponding delay
                    digitalWrite(frontDownRelayPin, LOW);
                    previousMillisFrontDownRelay=millis();
                    // Assure that the contrary is OFF (high)
                    digitalWrite(frontUpRelayPin, HIGH);
                    #ifdef SerialDEBUG
                      Serial.println("Lets deflate Front DOWN!");
                    #endif
#ifdef useNEXTION
                    sendCommand("texHFront.pco=0xB000"); // Dark Red
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
                }
                else{
                    if(millis() - previousMillisFrontDownRelay > delayRelay){
                        digitalWrite(frontDownRelayPin, HIGH); // turn it OFF
                    #ifdef SerialDEBUG
                        Serial.println("Stop deflating Front.");
                    #endif
                    }
                }
          }   // end if (Van1.desHFront<Van1.HFront)
      } // end if Van1.desHFront out of tolerance)
     else {  
              digitalWrite(frontUpRelayPin, HIGH); // turn it OFF
              digitalWrite(frontDownRelayPin, HIGH); // turn it OFF
              Van1.flagFront=1; // Front height is correct!
              #ifdef SerialDEBUG
                  Serial.println("FRONT HEIGHT IS CORRECT");
              #endif
#ifdef useNEXTION
              sendCommand("texHFront.pco=BLACK");
#endif
     }
 
 
    if(Van1.flagFront&&Van1.flagRear){
        Van1.letsControlHeight=0; // everything is correct
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
      Van1.readPressures_Nsamples(30);
      Van1.checkPComp();

      // if we have not deflated before...
      if (Van1.flagDeflated==0){
          if(Van1.PFront>PLowerThresh){ // deflating front suspension
               if(digitalRead(frontDownRelayPin)){ 
                    #ifdef SerialDEBUG
                    Serial.println("Deflating front suspension.");
                    #endif
// if it is HIGH (off), activate it and save the change time
                // Take care of inverted logic with the relay module
                    // Activate corresponding delay
                    digitalWrite(frontDownRelayPin, LOW);
                    previousMillisFrontDownRelay=millis();
                    // Assure that the contrary is OFF (high)
                    digitalWrite(frontUpRelayPin, HIGH);                  
#ifdef useNEXTION
                    sendCommand("texHFront.pco=0xB000"); // Dark Red
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
                }
                else{
                    if(millis() - previousMillisFrontDownRelay > delayRelay){
                        digitalWrite(frontDownRelayPin, HIGH); // turn it OFF
                    }
                }
          }
          else{
          digitalWrite(frontDownRelayPin, HIGH); // turn it OFF// closing front deflating valve
#ifdef useNEXTION
          sendCommand("texHFront.pco=BLACK"); // BLACK
#endif
          }
          
          if(Van1.PRear>PLowerThresh){ // deflating rear suspension

                if(digitalRead(rearDownRelayPin)){ // if it is HIGH (off), activate it and save the change time
                
                    #ifdef SerialDEBUG
                    Serial.println("Deflating rear suspension.");
                    #endif// Take care of inverted logic with the relay module
                    // Activate corresponding delay
                    digitalWrite(rearDownRelayPin, LOW);
                    previousMillisRearDownRelay=millis();
                    // Assure that the contrary is OFF (high)
                    digitalWrite(rearUpRelayPin, HIGH);
                    
#ifdef useNEXTION
                    sendCommand("texHRear.pco=0xB000"); // Dark Red
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
                }
                else{
                    if(millis() - previousMillisRearDownRelay > delayRelay){
                        digitalWrite(rearDownRelayPin, HIGH); // turn it OFF
                    }
                }
          }
          else{
          digitalWrite(rearDownRelayPin, HIGH); // turn it OFF // closing front deflating valve
#ifdef useNEXTION
          sendCommand("texHRear.pco=BLACK"); // BLACK
#endif
          }
        
        //if both are deflated turn flag ON
          if((Van1.PRear<PLowerThresh)&&(Van1.PFront<PLowerThresh)){
              Van1.flagDeflated=1;  // deflating finished      
        }
     
      }// end if Van1.flagDeflated==0



     if (Van1.flagDeflated==1){
        
        // Reading sensors again
        Van1.readAngle_Nsamples(60);
        Van1.readPressures_Nsamples(30);
        Van1.checkPComp();
 
     // Controlling positive angles
     // Being a positive angle one with the van's front upper than its rear
        if(Van1.angle>tolAngle){ // if it is positive we have to inflate rear
//                PRear++; // increase Rear Pressure for simulation

           if((!Van1.flagCompDeflated)&&(Van1.flagCompInflated)){// then we can increase Pressure
//           if(Van1.PComp>CompDeflatedThresh){// then we can increase Pressure
                 
                if(digitalRead(rearUpRelayPin)){ // if it is HIGH (off), activate it and save the change time
                // Take care of inverted logic with the relay module
                    // Activate corresponding delay
                    digitalWrite(rearUpRelayPin, LOW);
                    previousMillisRearUpRelay=millis();
                    // Assure that the contrary is OFF (high)
                    digitalWrite(rearDownRelayPin, HIGH); 
#ifdef useNEXTION
                    sendCommand("texHRear.pco=0x0408"); // Dark Green
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
                }
                else{
                    if(millis() - previousMillisRearUpRelay > delayRelay){
                        digitalWrite(rearUpRelayPin, HIGH); // turn it OFF
                    }
                }
           }
           else{  // else if compRead<CompDeflatedThresh we cannot increase Pressure
                 digitalWrite(rearUpRelayPin, HIGH); // turn it OFF// closing rear inflating valve
#ifdef useNEXTION
                 sendCommand("vis texMessage,1"); // Show texMessage
                 sendCommand("texHRear.pco=21087"); // Light Blue
                 sendCommand("texMessage.txt=\"Low compressor pressure.\""); // Change text
#endif
            }// end if compRead>CompDeflatedThresh
        }
        else{ // if not a possitive angle I can close rear inflating valve
           digitalWrite(rearUpRelayPin, HIGH); // turn it OFF// close inflating rear valve
#ifdef useNEXTION
           sendCommand("texHRear.pco=BLACK"); // BLACK
#endif
       }
                            
      // for negative angles we increase front Pressure
      if(Van1.angle<tolAngle){ // if it is negative we have to inflate front
           if((!Van1.flagCompDeflated)&&(Van1.flagCompInflated)){// then we can increase Pressure
//            if(Van1.PComp>CompDeflatedThresh){// then we can increase Pressure
               
                 if(digitalRead(frontUpRelayPin)){ // if it is HIGH (off), activate it and save the change time
                // Take care of inverted logic with the relay module
                    // Activate corresponding delay
                    digitalWrite(frontUpRelayPin, LOW);
                    previousMillisFrontUpRelay=millis();
                     // Assure that the contrary is OFF (high)
                    digitalWrite(frontDownRelayPin, HIGH);
#ifdef useNEXTION
                    sendCommand("texHFront.pco=0x0408"); // Dark Green
                    sendCommand("texMessage.txt=\"\""); // Blank text
#endif
                }
                else{
                    if(millis() - previousMillisFrontUpRelay > delayRelay){
                        digitalWrite(frontUpRelayPin, HIGH); // turn it OFF
                    }
                }
            }
            else{  // else if compRead<CompDeflatedThresh we cannot increase Pressure
               digitalWrite(frontUpRelayPin, HIGH); // turn it OFF// closing rear inflating valve
#ifdef useNEXTION
               sendCommand("vis texMessage,1"); // Show texMessage
               sendCommand("texHFront.pco=21087"); // Light Blue
               sendCommand("texMessage.txt=\"Low compressor pressure.\""); // Change text
#endif
            }// end if compRead>CompDeflatedThresh
        }
      else{ // if not a possitive angle I can close rear inflating valve
           digitalWrite(frontUpRelayPin, HIGH); // turn it OFF// close inflating rear valve
#ifdef useNEXTION
           sendCommand("texHFront.pco=BLACK"); // BLACK
#endif
        }

     } // end if Deflated=1

    if(abs(Van1.angle)<tolAngle){
        Van1.letsControlAngle=0; // everything is correct!
    }


    // Reading sensors after control
//    readheigths_Nsamples(30);
    Van1.readAngle_Nsamples(40);
    Van1.checkAngle();
    Van1.readPressures_Nsamples(30);
    Van1.checkPComp();


#ifdef useNEXTION
 // refresh values after control
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
       
  } // end if Van1.letsControlAngle
  else if (!Van1.letsControlHeight) {
    Van1.turnAllRelaysOFF();
  }


} // end if millis control


  
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


///////////////// NEXTION DISPLAY UPDATE SECTION /////////////////////////////
/*    // Sending data via nextion LCD screen
  currentMillis = millis();  
  if (currentMillis - previousMillisLCDPrint >= LCDprintInterval){
    previousMillisLCDPrint = currentMillis;
  
   }
*/


  //delay(20); // bit of delay...
  
}  //end main loop
