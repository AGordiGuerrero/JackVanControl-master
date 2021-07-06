
/*
  JackVanControl.cpp - Library for Jack's van automatic control
  Created by A.Gordillo-Guerrero, May, 2021.
  Released into the public domain.
*/

//#include "Arduino.h"
#include "JackVanControl.h"
#include "Wire.h" // for IMU 6050 control
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro; // giroscope object

//#define SerialDEBUG

#define useNEXTION // for using Nextion HMI tactile screen

#ifdef useNEXTION
#include "Nextion.h"
#include "HardwareSerial.h"
#endif


//////////////////////////////////
// Constructor //////////////////
//JackVan::JackVan(int VbatPin)
JackVan::JackVan(int buttonPin, int switchPin, int VbatPin, int CompPin, int rearPressurePin, int frontPressurePin, int rearUpRelayPin, int rearDownRelayPin, int frontUpRelayPin, int frontDownRelayPin)
{

  // Pin mode definition
  // Relay board outputs
  pinMode(rearUpRelayPin   , OUTPUT);
  pinMode(rearDownRelayPin , OUTPUT);
  pinMode(frontUpRelayPin  , OUTPUT);
  pinMode(frontDownRelayPin, OUTPUT);

  // private variable asignation
  _buttonPin=buttonPin;
  _switchPin=switchPin;
  _VbatPin=VbatPin;
  _CompPin=CompPin;
  _rearPressurePin=rearPressurePin;
  _frontPressurePin=frontPressurePin;
  _rearUpRelayPin=rearUpRelayPin;
  _rearDownRelayPin=rearDownRelayPin;
  _frontUpRelayPin=frontUpRelayPin;
  _frontDownRelayPin=frontDownRelayPin;


}



///////////////////////////////////
// Initializator //////////////////
//   To assure the correct startup of modules
///////////////////////////////////

void JackVan::init()
{


  //setting initial mode to MANUAL
  mode=4;

  // Initializing serial communication
  Serial.begin(9600); //Start Serial monitor in 9600

  // Turn OFF all relays at start
  turnAllRelaysOFF();

#ifdef useNEXTION

  Serial2.begin(9600); // Serial comunication with Nextion display using serial2 port
  // Nextion definitions
  nexInit();

  sendCommand("vis texMessage,0"); // Hide texMessage at startup

#endif

  // IMU MCU6050 initialisation
  Wire.begin(); // 21=SDA / 22=SCL
  accelgyro.initialize();

//#ifdef SerialDEBUG
	Serial.println("Testing MPU6050 connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//#endif

//setting offsets obtained for each MPU6050 module using Examples-> MCU6050-> IMU_Zero.ino
// with th PCB located in real van, at 20210612
  accelgyro.setXGyroOffset(55);
  accelgyro.setYGyroOffset(-16);
  accelgyro.setZGyroOffset(-3);
  accelgyro.setXAccelOffset(-2140);
  accelgyro.setYAccelOffset(2826);
  accelgyro.setZAccelOffset(5672);  

// Checking for strange values of sensors at startup

Serial.println("Checking sensor values...");

// Battery check...
Serial.println("Checking battery...");
delay(1000);
readVbat();
checkVbat(); // setting Battery flags
if(Vbat<VbatLowerThresh){ Serial.println("Take care. Low battery value."); }
else if (Vbat>VbatUpperThresh){Serial.println("Take care. High battery value."); }
else{Serial.println("Battery ok!" ); }

// Angle check...
Serial.println("Checking angle...");
delay(1000);
readAngle_Nsamples(50);
checkAngle(); // setting Angle flag
if(angle<-AngleUpperThresh){ Serial.println("Take care. Low angle value."); }
else if (angle>AngleUpperThresh){Serial.println("Take care. High angle value."); }
else{Serial.println("Angle ok!" ); }

// Pressure sensors
Serial.println("Checking pressures...");
delay(1000);
readPressures_Nsamples(40);
checkPComp(); // setting Compressor flags
if((PComp<0)||(PComp>180)){ Serial.println("Take care. Strange compressor pressure."); }
else {Serial.println("Compressor pressure ok!" ); }
if((PFront<0)||(PFront>180)){ Serial.println("Take care. Strange Front pressure."); }
else {Serial.println("Front pressure ok!" ); }
if((PRear<0)||(PRear>180)){ Serial.println("Take care. Strange Rear pressure."); }
else {Serial.println("Rear pressure ok!" ); }

// Heights sensors
Serial.println("Measuring heights...");
delay(1000);
readHeights_Nsamples_discard(60);
Serial.println("Checking heights...");
checkHeights();
if((HFront<HeightLowerThresh)||(HFront>HeightUpperThresh)){ Serial.println("Take care. Strange Front height."); ; Serial.print("HFront= "); Serial.print(HFront);}
else {Serial.println("Front height ok!" ); }
if((HRear<HeightLowerThresh) ||(HRear>HeightUpperThresh)){ Serial.println("Take care. Strange Rear height."); Serial.print("HRear= "); Serial.print(HRear); }
else {Serial.println("Rear height ok!" ); }

Serial.println();

}



void JackVan::getStatus()
{

	Serial.println("  Control variables:");
	Serial.print("    Current Mode: "); Serial.println(modes[mode]);
	Serial.print("       Desired Heights (cm): "); Serial.print("     Front: "); Serial.print(desHFront);  Serial.print("   Rear: ");  Serial.println(desHRear);
  
  Serial.print("    Control action: "); 
  if( (letsControlHeight)){ Serial.println("     Trying to control heights."); }
    else if(letsControlAngle){ Serial.println("    Trying to control angle."); }
      else{Serial.println("      Not controlling anything." ); }
  if(flagHRear){ Serial.println("      Rear height is correct"); }
      else{Serial.println("     Rear height is NOT correct." ); }
  if(flagHFront){ Serial.println("     Front height is correct"); }
      else{Serial.println("     Front height is NOT correct." ); }

	Serial.println();

	Serial.println("  Van's physical variable:");
	Serial.println("    Pressures (PSI): ");
  Serial.print("       Front: "); Serial.print(PFront);
  Serial.print("   Rear: ");  Serial.print(PRear);
  Serial.print("   Compressor: "); Serial.println(PComp);
  if(flagCompInflated){ Serial.println("      Compressor is inflated."); }
      else { Serial.println("      Compressor is not inflated."); }
      
  if(flagCompDeflated){Serial.println("      Compressor is deflated." ); }
      else { Serial.println("      Compressor is not deflated."); }

	Serial.println("    Heights (cm): ");
  Serial.print("       Front: "); Serial.print(HFront);
  Serial.print("   Rear: ");  Serial.println(HRear);
  if(flagHeights){ Serial.println("      Heigths are in measurement range."); }
      else {Serial.println("     Take care: heights are not in measurement range." ); }

  Serial.print("    Angle (Degrees): "); Serial.println(angle);
  if(flagAngle){ Serial.println("      Angle is in measurement range."); }
      else {Serial.println("     Take care: angle is not in measurement range." ); }

  Serial.print("    Battery Voltage (V): "); Serial.println(Vbat);
  if(flagVbat){ Serial.println("    Voltage battery is in action range."); }
      else {Serial.println("     Take care: Voltage battery is not in action range." ); }

	Serial.println();
}

void JackVan::readandcheckAll_Nsamples(int Nsamples)
{
    readVbat();
    checkVbat();
    readPressures_Nsamples(Nsamples/2); // it is enough with Nsamples/2
    checkPComp();
    readHeights_Nsamples_discard(Nsamples);
    checkHeights();
    readAngle_Nsamples(Nsamples);   
    checkAngle();
}



void JackVan::readTouchbutton()
{
  // Reading variables from th capacitive button
  int touchValue = touchRead(_buttonPin);
  //Serial.print("TouchValue: ");
  //Serial.println(touchValue);
  // check if the touchValue is below the threshold
  // if it is, set ledPin to HIGH
  if(touchValue < CapThresh){ 

    if (flagCapDebounce)
      analogbuttonpressed();
    
    flagCapDebounce=1; // flag 1 when the capacitive touch button is bellow threshold
    
  }
  else{flagCapDebounce=0;} // to avoid undessired events

}






void JackVan::readVbat()
{
  
  Vbat = analogRead(_VbatPin); //reading pin previously defined
    // cuadratic fitting
    // f(x)=5.58E-06*x*x - 0.00664*x +6.685
    // f(x)=4.02E-06*x*x - 0.00277*x +4.522
  Vbat = round(4.02E-06*Vbat*Vbat - 0.00277*Vbat +4.522);

#ifdef SerialDEBUG
  Serial.print("Battery voltaje (V): ");
  Serial.println(Vbat);
#endif
  
}

void JackVan::readPressures_onesample()
{

   PComp = analogRead(_CompPin);
    // linear fitting using "MedidasPresion_20210314.ods"
    // f(x)= 0.06159*x -23.4666
   PComp = round(0.06159*PComp -23.4666);
 
   PRear = analogRead(_rearPressurePin);
    // linear fitting using "MedidasPresion_20210314.ods"
    // f(x)= 0.04229*x - 9.48432
   PRear = round(0.04229*PRear - 9.48432);

   PFront = analogRead(_frontPressurePin);
    // linear fitting using "MedidasPresion_20210314.ods"
    // f(x)= 0.04199*x - 8.6636
   PFront = round(0.04199*PFront - 8.6636);

#ifdef SerialDEBUG
   	Serial.print("Compressor Pressure: ");     	Serial.println(PComp);
   	Serial.print("Front Pressure: ");    	Serial.println(PFront);
    	Serial.print("Rear Pressure:   ");    	Serial.println(PRear);
#endif

}


void JackVan::readPressures_Nsamples(int Nsamples)
{
	int8_t   i;

  	float averageP=0; //sample average
  	int tmp; //temporal
  
  	//read until Nsamples are obtained, or readcount_max trial measures have been performed
  	for (i=0; i < Nsamples; i++){      
    		tmp = analogRead(_CompPin);
   	 	// linear fitting using "MedidasPresion_20210314.ods"
    		// f(x)= 0.06159*x -23.4666
    		averageP += round(0.06159*tmp -23.4666);
	}
  	PComp = round(averageP/Nsamples);


  	averageP=0; //reseting cumulator
  	for (i=0; i < Nsamples; i++){      
    		tmp = analogRead(_rearPressurePin);
    		// linear fitting using "MedidasPresion_20210314.ods"
    		// f(x)= 0.04229*x - 9.48432
    		averageP += round(0.04229*tmp - 9.48432);
    	}
  	PRear = round(averageP/Nsamples);
    
  	averageP=0; //reseting cumulator
  	for (i=0; i < Nsamples; i++){      
    		tmp = analogRead(_frontPressurePin);
    		// linear fitting using "MedidasPresion_20210314.ods"
    		// f(x)= 0.04199*x - 8.6636
    		averageP += round(0.04199*tmp - 8.6636);
  	}
  	PFront= round(averageP/Nsamples);
  
#ifdef SerialDEBUG
	Serial.print("Average Compressor Pressure: "); 	Serial.println(PComp);
    	Serial.print("Average Front Pressure: "); 	Serial.println(PFront);
    	Serial.print("Average Rear Pressure:   "); 	Serial.println(PRear);
#endif

}


void JackVan::readAngle_onesample()
{

  int16_t xa, ya, za;
//  float roll;
  float pitch;
  
  // Estimate angle values from acceleration values from IMU using i2cdev and MCU6050 libraries
  accelgyro.getAcceleration(&xa, &ya, &za);
  //roll = atan2(ya , za) * 180.0 / PI;
  pitch = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI;


  angle=pitch;
  
#ifdef SerialDEBUG
  //Serial.print("Roll: "); 
  //Serial.print(roll);
  //Serial.print(",");
  Serial.print("Angle: ");
  Serial.println(angle);
#endif

}


void JackVan::readAngle_Nsamples(int Nsamples)
{

  int16_t xa, ya, za;
  //float roll;
  float pitch;
  
  int8_t   i;
  float averagePitch=0; //sample average
  //float averageRoll=0; //sample average
  
  //read until Nsamples are obtained, or readcount_max trial measures have been performed
  for (i=0; i < Nsamples; i++){      

  // Estimate angle values from acceleration values from IMU using i2cdev and MCU6050 libraries
  
	accelgyro.getAcceleration(&xa, &ya, &za);
  	//roll = atan2(ya , za) * 180.0 / PI;
  	pitch = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI;
      averagePitch+=pitch; //sample average
      //averageRoll+=roll; //sample average
  } // end for Nsamples
   

  averagePitch/=(float)Nsamples;
  //averageRoll/=(float)Nsamples;
    
  angle=averagePitch;

  
#ifdef SerialDEBUG
    //Serial.print("Roll = ");
    //Serial.println(roll,1);
    	Serial.print("Average angle (degrees)= ");
	Serial.println(angle,1);
#endif


}

void JackVan::readAngle_Nsamples_discard(int Nsamples)
{

  int8_t   i;
  int8_t   Ngood=0;
  int16_t xa, ya, za;

  float pitch;
  float averagePitch=0; //sample average
  //float roll;
  //float averageRoll=0; //sample average
  
  //read until Nsamples are obtained, or readcount_max trial measures have been performed
  for (i=0; i < Nsamples; i++){      

  // Estimate angle values from acceleration values from IMU using i2cdev and MCU6050 libraries
  
	  accelgyro.getAcceleration(&xa, &ya, &za);
  	//roll = atan2(ya , za) * 180.0 / PI;
  	pitch = atan2(-xa , sqrt(ya * ya + za * za)) * 180.0 / PI;


	// only count measures in range
	  if ((pitch > -AngleUpperThresh ) && ( pitch < AngleUpperThresh)){
		  Ngood++;
  		averagePitch+=pitch; //sample average
	  }

} // end for Nsamples
   
averagePitch/=(float)Ngood;
  //averageRoll/=(float)Nsamples;
    
angle=averagePitch;

  
#ifdef SerialDEBUG
    //Serial.print("Roll = ");
    //Serial.println(roll,1);
    	Serial.print("Average angle (degrees)= ");
	Serial.println(angle,1);
#endif


}





void JackVan::readHeights_onesample()
{

  // Reading TOFsensor1 (Front)
  TOFSensorRead(i2c_rx_buf, TOF1_I2Caddress);
  length_val=i2c_rx_buf[0];
  length_val=length_val<<8;
  length_val|=i2c_rx_buf[1];
  HFront=round((float)length_val/10.); // changing to cm

  // Reading TOFsensor2 (Rear)
  TOFSensorRead(i2c_rx_buf, TOF2_I2Caddress);
  length_val=i2c_rx_buf[0];
  length_val=length_val<<8;
  length_val|=i2c_rx_buf[1];
  HRear=round((float)length_val/10.); // changing to cm
  
  #ifdef SerialDEBUG
    	Serial.print("Front height (cm): "); 	Serial.println(HFront);
	Serial.print("Rear height (cm)"); 
	Serial.println(HRear);
  #endif

}



void JackVan::readHeights_Nsamples(int Nsamples)
{

  int8_t   i;

  float averageH=0; //sample average
  
  // Reading TOFsensor1 (Front)
  //read until Nsamples are obtained
  for (i=0; i < Nsamples; i++){      
  	TOFSensorRead(i2c_rx_buf, TOF1_I2Caddress);
  	length_val=i2c_rx_buf[0];
  	length_val=length_val<<8;
  	length_val|=i2c_rx_buf[1];
  	averageH+=(float)length_val/10.; // changing to cm
  }
  HFront=round(averageH/(float)Nsamples);


  averageH=0; //reseting cumulator
  // Reading TOFsensor2 (Rear)
  //read until Nsamples are obtained
  for (i=0; i < Nsamples; i++){      
      TOFSensorRead(i2c_rx_buf, TOF2_I2Caddress);
      length_val=i2c_rx_buf[0];
      length_val=length_val<<8;
      length_val|=i2c_rx_buf[1];
	averageH+=(float)length_val/10.; // changing to cm
  }
  HRear=round(averageH/(float)Nsamples);
  
  #ifdef SerialDEBUG
  	Serial.print("Average Front height (cm): "); 	Serial.println(HFront);
    	Serial.print("Average Rear height (cm)"); 	Serial.println(HRear);
  #endif

}

void JackVan::readHeights_Nsamples_discard(int Nsamples)
{

  int8_t   i;
  int8_t   Ngood=0;

  float averageH=0; //sample average
  
  // Reading TOFsensor1 (Front)
  //read until Nsamples are obtained but discard strange ones
  for (i=0; i < Nsamples; i++){      
  	TOFSensorRead(i2c_rx_buf, TOF1_I2Caddress);
  	length_val=i2c_rx_buf[0];
  	length_val=length_val<<8;
  	length_val|=i2c_rx_buf[1];
	
	// only count measures in range
	  if ((length_val>HeightLowerThresh*10)&&(length_val<HeightUpperThresh*10)){
		  Ngood++;
  		averageH+=(float)length_val/10.; // changing to cm
	  }
  }
  HFront=round(averageH/(float)Ngood);


  averageH=0; //reseting cumulator
  Ngood=0; //reseting cumulator
  // Reading TOFsensor2 (Rear)
  //read until Nsamples are obtained
  for (i=0; i < Nsamples; i++){      
      TOFSensorRead(i2c_rx_buf, TOF2_I2Caddress);
      length_val=i2c_rx_buf[0];
      length_val=length_val<<8;
      length_val|=i2c_rx_buf[1];

	// only count measures in range
	    if ((length_val>HeightLowerThresh*10)&&(length_val<HeightUpperThresh*10)){
		    Ngood++;
  		  averageH+=(float)length_val/10.; // changing to cm
    	}
  }
  HRear=round(averageH/(float)Ngood);
  
  #ifdef SerialDEBUG
  	Serial.print("Average Front height (cm): "); 	Serial.println(HFront);
    	Serial.print("Average Rear height (cm)"); 	Serial.println(HRear);
  #endif

}



void JackVan::TOFSensorRead(unsigned char* datbuf, int TOF_I2Caddress)
{

  //  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(TOF_I2Caddress); // transmit to device #82 (0x52) by default
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(0x00));      // sets distance data address (0x00)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(TOF_I2Caddress, 2);    // request 2 bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (2 <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }

}


void JackVan::analogbuttonpressed() {

//  previousMillisPush=millis();
  flagDim=0; // to dim after a while

  sprintf(buff, "dim=%d", 100); //set screen bright to maximum (dim=100)
  sendCommand(buff);

  // Always hide manual control buttons on screen
  sendCommand("vis butUpFront,0"); // Hide button
  sendCommand("vis butDownFront,0"); // Hide button
  sendCommand("vis butUpRear,0"); // Hide button
  sendCommand("vis butDownRear,0"); // Hide button

  uint16_t reading = analogRead(_switchPin);
  // print out the values you read:
  //sprintf(buff, "%d  \n", reading);
  //Serial.println(buff);

  // make correspondence between read value and switch position
  // it is composed of a simple voltage divider
  if(reading<800){
    Serial.println("Changing to Mode AUTO");
    mode=0; 
    letsControlHeight=0; // we do not control height
    letsControlAngle=1; // we do control angle
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
    flagDeflated=0; // to start deflating
//    texMode0.setText("AUTOLEVEL MODE");
#ifdef DEBUG_SERIAL_ENABLE
  dbSerial.println("Auto Mode ON");
#endif
// in auto mode we choose the parameters of the Parking mode
    desHFront=desHParkFront;
    desHRear=desHParkRear;
    desPFront=desPParkFront;
    desPRear=desPParkRear;
  }
  else if(reading<1800){
    Serial.println("Changing to Mode PARK"); 
    mode=1; 
    letsControlHeight=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
//    texMode0.setText("PARK MODE");
    desHFront=desHParkFront;
    desHRear=desHParkRear;
    desPFront=desPParkFront;
    desPRear=desPParkRear;
  }
  else if(reading<3000){
    Serial.println("Changing to Mode ROAD"); 
    mode=2; 
    letsControlHeight=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
   //   texMode0.setText("ROAD MODE");
    desHFront=desHRoadFront;
    desHRear=desHRoadRear;
    desPFront=desPRoadFront;
    desPRear=desPRoadRear;

  }
  else {
    Serial.println("Changing to Mode TRAIL"); 
    mode=3; 
    letsControlHeight=1; // we have to control height
    letsControlAngle=0; // we do not control angle
    NTrialsFront=0; // Put to zero the number of control trials
    NTrialsRear=0; // Put to zero the number of control trials
  //    texMode0.setText("TRAIL MODE");
    desHFront=desHTrailFront;
    desHRear=desHTrailRear;
    desPFront=desPTrailFront;
    desPRear=desPTrailRear;
  }

}


void JackVan::checkVbat()
{

  if( (Vbat<VbatLowerThresh) || (Vbat>VbatUpperThresh) ){ flagVbat=0; }
  else { flagVbat=1; }

}


void JackVan::checkPComp()
{

// checking correct ranges
   if ((PComp>PressLowerThresh) && (PComp<PressUpperThresh)) { flagPComp=1; }
     else{ flagPComp=0; }

   if ((PFront>PressLowerThresh) && (PFront<PressUpperThresh)) { flagPFront=1; }
     else{ flagPFront=0; }
   
   if ((PRear>PressLowerThresh) && (PRear<PressUpperThresh)) { flagPRear=1; }
     else{ flagPRear=0; }

// checking infation proccess
  if(PComp<CompDeflatedThresh){ flagCompDeflated=1;  flagCompInflated=0; }

  if(PComp>CompInflatedThresh){ flagCompInflated=1;  flagCompDeflated=0; }

}


void JackVan::checkHeights()
{

  if((HFront >= HeightLowerThresh) && (HFront <= HeightUpperThresh))
      { flagHFront=1; }
      else { flagHFront=0; }
  
  if((HRear >= HeightLowerThresh) && (HRear <= HeightUpperThresh))
      { flagHRear=1; }
      else { flagHRear=0; }
  
  if(    (HFront<HeightLowerThresh) || (HFront>HeightUpperThresh)
      || (HRear<HeightLowerThresh)  || (HRear>HeightUpperThresh))
      { flagHeights=0; }
      else
      { flagHeights=1; }

}



void JackVan::checkAngle()
{

      if( (angle < -AngleUpperThresh) || (angle > AngleUpperThresh) ) { flagAngle=0; }
      else
      { flagAngle=1; }

}


void JackVan::updateNextionValues()
{

    char buff[50];

  // Updating Nextion screen text fields avoiding strange values
     // displaying Vbat
    if(flagVbat){
    sprintf(buff, "%.1fV", Vbat); // displaying Vbat
    }
    else{ sprintf(buff, "--V");}
    texvalVbat.setText(buff);
    // displaying PRear
    if(flagPRear){
    utoa(PRear, buff, 10); // unsigned integer to char
    }
    else{ sprintf(buff, "--"); } 
    texvalRear0.setText(buff);
    // displaying PFront
    if(flagPFront){
    utoa(PFront, buff, 10); // unsigned integer to char
    }
    else{ sprintf(buff, "--"); } 
    texvalFront0.setText(buff);
    // displaying PComp
    if(flagPComp){
    utoa(PComp, buff, 10); // unsigned integer to char
    }
    else{ sprintf(buff, "--"); } 
    texvalComp0.setText(buff);
    // displaying Angle
    if(flagAngle){
    sprintf(buff, "%.1f\xB0", angle); // displaying º symbol
    }
    else{ sprintf(buff, "--\xB0"); } 
    texvalAngle0.setText(buff);
    // displaying rear height
    if(flagHRear){
    sprintf(buff, "%d / %d cm", HRear, desHRear);
    }
    else{ sprintf(buff, "-- / %d cm", desHRear);} 
    texHRear0.setText(buff);
    // displaying front height
    if(flagHFront){
    sprintf(buff, "%d / %d cm", HFront, desHFront);
    }
    else{ sprintf(buff, "-- / %d cm", desHFront);} 
    texHFront0.setText(buff);
    
    // displaying text MODE
    switch (mode)
    {
    case 0:
      texMode0.setText("AUTO");
      break;
    case 1:
      texMode0.setText("PARK");
      break;
    case 2:
      texMode0.setText("ROAD");
      break;
    case 3:
      texMode0.setText("TRAIL");
      break;
    case 4:
      texMode0.setText("MANUAL");    
      break;
    default:
      texMode0.setText("MANUAL");    
      break;
    } 
}

void JackVan::turnAllRelaysOFF()
{
 
   // Turn OFF all relays
  digitalWrite(_rearUpRelayPin, HIGH);
  digitalWrite(_rearDownRelayPin, HIGH);
  digitalWrite(_frontUpRelayPin, HIGH);
  digitalWrite(_frontDownRelayPin, HIGH);
  
}
