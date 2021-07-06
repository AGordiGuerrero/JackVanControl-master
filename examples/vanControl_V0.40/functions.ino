
// VOLKSWAGEN VAN CONTROL FOR JACOBO MONTAÑO FERNÁNDEZ (JackVanControl)
//---------------------******--------------------------//
//
// Included functions in the main code, and not included in the JackVan-master library.
//
//
//
//


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
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
  letsControlHeight=1; // we have to control height
  letsControlAngle=0; // we do not control angle
  texMode0.setText("PARKING MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Parking Mode ON");
#endif
  desHFront=desHParkFront;
  desHRear=desHParkRear;
}

void butRoad0Callback(void *ptr) {
  // saving push time
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
  letsControlHeight=1; // we have to control height
  letsControlAngle=0; // we do not control angle
  texMode0.setText("ROAD MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Road Mode ON");
#endif
  desHFront=desHRoadFront;
  desHRear=desHRoadRear;
}


void butAuto0Callback(void *ptr) {
  // saving push time
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
  letsControlHeight=0; // we do not control height
  letsControlAngle=1; // we do control angle
  flagDeflated=0; // to start deflating
  texMode0.setText("AUTOLEVEL MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Auto Mode ON");
#endif
// in auto mode we choose the parameters of the Parking mode
  desHFront=desHParkFront;
  desHRear=desHParkRear;
}


void butTrail0Callback(void *ptr) {
  // saving push time
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
  letsControlHeight=1; // we have to control height
  letsControlAngle=0; // we do not control angle
  texMode0.setText("TRAIL MODE");
  // Hidding manual control buttons
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Trail Mode ON");
#endif 
  desHFront=desHTrailFront;
  desHRear=desHTrailRear;
}


void butManual0Callback(void *ptr) {
  // saving push time
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
  letsControlHeight=0; // we do not control height
  letsControlAngle=0; // we do not control angle
  texMode0.setText("MANUAL MODE");
  // Showing manual control buttons
  sendCommand("vis butUpFront,1"); // Show button
  sendCommand("vis butDownFront,1"); // Show button
  sendCommand("vis butUpRear,1"); // Show button
  sendCommand("vis butDownRear,1"); // Show button

#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Manual Mode ON");
#endif
  desHFront=200;
  desHRear=200;

}

void butUpFront0Callback(void *ptr) {
  // saving push time
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
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
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
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
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
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
  previousMillisPush=millis();
  flagDim=0; // to dim after a while
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
