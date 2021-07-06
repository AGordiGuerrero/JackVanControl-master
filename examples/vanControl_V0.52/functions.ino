
// VOLKSWAGEN VAN CONTROL FOR JACOBO MONTAÑO FERNÁNDEZ (JackVanControl)
//---------------------******--------------------------//
//
// Included functions in the main code, and not included in the JackVan-master library.
//

#define useNEXTION // for using Nextion HMI tactile screen


// Function to turn OFF all relays if it is time to do it.
 void tryControlRearHeight(){
 
        // First we control if desired height is smaller than current height
   if(Van1.desHRear<Van1.HRear){//we have to decrease Rear Height
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
    }    
    else {  // end if(Van1.desHRear<Van1.HRear)
           // Deactivate corresponding delay (turn input to HIGH)
             digitalWrite(rearDownRelayPin, HIGH);
    }

         // Next we control if desired height is bigger than current height
    if(Van1.desHRear>Van1.HRear){//we have to increase Rear Height

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
            } // end if pin was HIGH
            
             
         } // end if Van1.desHRear>Van1.HRear
         else {
           // Deactivate corresponding delay (turn input to HIGH)
             digitalWrite(rearUpRelayPin, HIGH);
         }
         
 }



 void  tryControlFrontHeight(){

  // First we control if desired height is smaller than current height
    if(Van1.desHFront<Van1.HFront){//we have to decrease Front Pressure
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
    }   // end if (Van1.desHFront<Van1.HFront)
    else {
           // Deactivate corresponding delay (turn input to HIGH)
             digitalWrite(frontDownRelayPin, HIGH);
    }

    // Next we control if desired height is bigger than current height
     if(Van1.desHFront>Van1.HFront){//we have to increase Front height

          #ifdef SerialDEBUG
          Serial.println("Trying to inflate Front UP!");
          #endif

                // Take care of inverted logic with the relay module
          if(digitalRead(frontUpRelayPin)){ // if it is OFF, turn it ON it and save the change time
          
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

         }
         else {
           // Deactivate corresponding delay (turn input to HIGH)
             digitalWrite(frontUpRelayPin, HIGH);
         }

}




 void tryDeflateBoth(){
 
     // Front suspension
     if(Van1.PFront>Van1.PLowerThresh){ // deflating front suspension
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
      }
      else{
        digitalWrite(frontDownRelayPin, HIGH); // turn it OFF// closing front deflating valve
        #ifdef useNEXTION
        sendCommand("texHFront.pco=BLACK"); // BLACK
        #endif
      }

     // Rear suspension      
      if(Van1.PRear>Van1.PLowerThresh){ // deflating rear suspension
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
      }
      else{
          digitalWrite(rearDownRelayPin, HIGH); // turn it OFF // closing front deflating valve
          #ifdef useNEXTION
          sendCommand("texHRear.pco=BLACK"); // BLACK
          #endif
      }
        
      //if both are deflated turn flag ON
      if((Van1.PRear<Van1.PLowerThresh)&&(Van1.PFront<Van1.PLowerThresh)){
              Van1.flagDeflated=1;  // deflating finished      
      }

 }


 void tryControlAngle(){
 
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
         
 }





// Function to turn OFF all relays if it is time to do it.
 void checkAllRelaysMillis(){

  
  if(!digitalRead(rearUpRelayPin)){ // if it is ON, turn it OFF after delayRelay (ms)
     if(millis() - previousMillisRearUpRelay > delayRelay){
         digitalWrite(rearUpRelayPin, HIGH); // turn OFF
         #ifdef SerialDEBUG
         Serial.println("Stop Inflating REAR UP.");
         #endif
     }
  }

 if(!digitalRead(rearDownRelayPin)){ // if it is ON, turn it OFF after delayRelay (ms)
      if(millis() - previousMillisRearDownRelay > delayRelay){
         digitalWrite(rearDownRelayPin, HIGH); // turn OFF
         #ifdef SerialDEBUG
         Serial.println("Stop Deflating REAR DOWN.");
         #endif
     }
  }

 if(!digitalRead(frontUpRelayPin)){ // if it is ON, turn it OFF after delayRelay (ms)
    if(millis() - previousMillisFrontUpRelay > delayRelay){
         digitalWrite(frontUpRelayPin, HIGH); // turn OFF
         #ifdef SerialDEBUG
         Serial.println("Stop Inflating FRONT UP.");
         #endif
     }
  }

  if(!digitalRead(frontDownRelayPin)){ // if it is ON, turn it OFF after delayRelay (ms)
     if(millis() - previousMillisFrontDownRelay > delayRelay){
         digitalWrite(frontDownRelayPin, HIGH); // turn OFF
         #ifdef SerialDEBUG
         Serial.println("Stop Deflating FRONT DOWN.");
         #endif
     }
  }

}
   

#ifdef useNEXTION


/// Button callbacks functions
// Button butPark0 component Push callback function. 
// When the Parking button is released, the text on the texMode changes and global variables set. 
void butPark0Callback(void *ptr) {
  // saving push time
  Van1.previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=1; // we have to control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=1; // setting mode
  //Van1.texMode0.setText("PARKING MODE");
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
  Van1.desPFront=Van1.desPParkFront;
  Van1.desPRear=Van1.desPParkRear;
}

void butRoad0Callback(void *ptr) {
  // saving push time
  Van1.previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=1; // we have to control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=2; // setting mode
  //Van1.texMode0.setText("ROAD MODE");
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
  Van1.desPFront=Van1.desPRoadFront;
  Van1.desPRear=Van1.desPRoadRear;
}



void butTrail0Callback(void *ptr) {
  // saving push time
  Van1.previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=1; // we have to control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=3; // setting mode
  //Van1.texMode0.setText("TRAIL MODE");
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
  Van1.desPFront=Van1.desPTrailFront;
  Van1.desPRear=Van1.desPTrailRear;

}



void butAuto0Callback(void *ptr) {
  // saving push time
  Van1.previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=0; // we do not control height
  Van1.letsControlAngle=1; // we do control angle
  Van1.flagDeflated=0; // to start deflating
  Van1.mode=0; // setting mode
  //Van1.texMode0.setText("AUTOLEVEL MODE");
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
  Van1.desPFront=Van1.desPParkFront;
  Van1.desPRear=Van1.desPParkRear;
}


void butManual0Callback(void *ptr) {
  // saving push time
  Van1.previousMillisPush=millis();
  Van1.flagDim=0; // to dim after a while
  Van1.letsControlHeight=0; // we do not control height
  Van1.letsControlAngle=0; // we do not control angle
  Van1.mode=4; // setting mode
  //Van1.texMode0.setText("MANUAL MODE");
  // Showing manual control buttons
  sendCommand("vis butUpFront,1"); // Show button
  sendCommand("vis butDownFront,1"); // Show button
  sendCommand("vis butUpRear,1"); // Show button
  sendCommand("vis butDownRear,1"); // Show button

#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Manual Mode ON");
#endif
  // we choose these values just for have someones. It is mode Manual.
  Van1.desHFront=Van1.desHRoadFront;
  Van1.desHRear=Van1.desHRoadRear;
  Van1.desPFront=Van1.desPRoadFront;
  Van1.desPRear=Van1.desPRoadRear;

}

void butUpFront0Callback(void *ptr) {
  // saving push time
  Van1.previousMillisPush=millis();
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
  Van1.previousMillisPush=millis();
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
  Van1.previousMillisPush=millis();
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
  Van1.previousMillisPush=millis();
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

#endif