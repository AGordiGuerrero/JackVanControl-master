
// VOLKSWAGEN VAN CONTROL FOR JACOBO MONTAÑO FERNÁNDEZ (JackVanControl)
//---------------------******--------------------------//
//
// Included functions in the main code, and not included in the JackVan-master library.
//
//
//
//


// Standard function to response the capacitive touch button reading the 4P3T switch voltaje
void analogbuttonpressed() {

  previousMillisPush=millis();
  flagDim=0; // to dim after a while

  sprintf(buff, "dim=%d", 100); //set screen bright to maximum (dim=100)
  sendCommand(buff);

  // Always hide manual control buttons on screen
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button


  uint16_t reading = analogRead(switchPin);
  // print out the values you read:
  //sprintf(buff, "%d  \n", reading);
  //Serial.println(buff);

  // make correspondence between read value and switch position
  // it is composed of a simple voltage divider
  if(reading<800){
    Serial.println("Changing to Mode AUTO"); 

    letsControlHeight=0; // we do not control height
    letsControlAngle=1; // we do control angle
    flagDeflated=0; // to start deflating

    texMode0.setText("AUTOLEVEL MODE");
  // Hidding manual control buttons
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Auto Mode ON");
#endif
// in auto mode we choose the parameters of the Parking mode
  desHFront=desHParkFront;
  desHRear=desHParkRear;
  }
  else if(reading<1800){
    Serial.println("Changing to Mode PARK"); 
    letsControlHeight=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    texMode0.setText("PARK MODE");
    desHFront=desHParkFront;
    desHRear=desHParkRear;
  }
  else if(reading<3000){
    Serial.println("Changing to Mode ROAD"); 
    letsControlHeight=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    texMode0.setText("ROAD MODE");
    desHFront=desHRoadFront;
    desHRear=desHRoadRear;
  }
  else {
    Serial.println("Changing to Mode TRAIL"); 
    letsControlHeight=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    texMode0.setText("TRAIL MODE");
    desHFront=desHTrailFront;
    desHRear=desHTrailRear;
  }

}


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

