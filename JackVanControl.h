/*x   
  JackVanControl.h - Library for Jack's van automatic control. Include both physical variables and control variables.
  Created by A.Gordillo-Guerrero, May, 2021.
  Released into the public domain.
*/

#ifndef JackVanControl_h
#define JackVanControl_h

//#include "Arduino.h"
#include "Wire.h" // for IMU 6050 control
#include "I2Cdev.h"
#include "MPU6050.h"

#define useNEXTION // for using Nextion HMI tactile screen

#ifdef useNEXTION
#include "Nextion.h"
#include "HardwareSerial.h"
#endif

class JackVan
{

 public:
   //////// Constructor //////////////////
   JackVan(int buttonPin, int switchPin, int VbatPin, int CompPin,int rearPressurePin, int frontPressurePin, int rearUpRelayPin, int rearDownRelayPin, int frontUpRelayPin, int frontDownRelayPin);
   
   ///////////////////////////////////////
   //////// Attributes  //////////////////
   ///////////////////////////////////////
   //// Physical variable ////
   // Battery voltage read value
   float Vbat;
   // pressure read values
   int PFront;
   int PRear;
   int PComp;
   // Height read values
   unsigned short length_val = 0;
   unsigned char i2c_rx_buf[16];
   int HFront;
   int HRear;
   int TOF1_I2Caddress=2; //(0x5 hex)
   int TOF2_I2Caddress=2; // (0x5 hex)
   //int TOF1_I2Caddress=82; //(0x52 hex)
   //int TOF2_I2Caddress=3; // (0x03 hex)
   // Angle variables
   float angle;
   // Mode variable
   //  - 0 : AUTO
   //  - 1 : PARK
   //  - 2 : ROAD
   //  - 3 : TRAIL
   //  - 4 : MANUAL
   
   //// Control variable ////
   uint8_t mode;
   //int delayRelay=1000; // in ms
   int previousMillisPush; // last time a button was pushed
   // Initialize 2D array to show the modes
   char modes[5][8] = { "AUTO", "PARK", "ROAD", "TRAIL", "MANUAL" };
   uint8_t letsControlHeight=0; // flag 1 when a MODE button was pushed, 0 when we finish control
   uint8_t letsControlAngle=0; // flag 1 when MODE AUTO button was pushed, 0 when we finish control
   uint8_t flagDeflated=0; // flag 1 when Front suspension was deflated
   uint8_t flagVbat=0; // flag 1 when battery voltage is correct to open relays (marked by VbatUpperThresh and VbatLowerThresh )
   uint8_t flagCorrectFront=0; // flag 1 when Front Height is in tolerance range
   uint8_t flagCorrectRear=0; // flag 1 when Rear Height is in tolerance range
   uint8_t flagHeights=0; // flag 1 when both height lectures are in correct read range (marked by HeightUpperThresh and HeightLowerThresh )
   uint8_t flagAngle=0; // flag 1 when angle lectures is in correct read range (marked by AngleUpperThresh )
   uint8_t flagHFront=0; // flag 1 when Front Height is in control range
   uint8_t flagHRear=0; // flag 1 when Rear Height is in control range
   uint8_t flagPComp=0; // flag 1 when compressor pressure is in control range
   uint8_t flagPFront=0; // flag 1 when Front pressure is in control range
   uint8_t flagPRear=0; // flag 1 when Rear pressure is in control range
   uint8_t flagDim=0; // flag 1 when screen is dimmed
   uint8_t flagCompInflated=0; // flag 1 when Compressor was inflated correctly
   uint8_t flagCompDeflated=0; // flag 1 when Compressor was deflated correctly
   uint8_t flagCapDebounce=0; // flag 1 when capacitive touch button was pressed in the previous read
   uint8_t NTrialsFront=0;
   uint8_t NTrialsRear=0;
   uint8_t NTrialsMax=10; // maximum number of control trials. 
   int CapThresh=30; // Threshold for capacitive button
   int CompInflatedThresh=110; // Threshold for compressor pressure when inflated
   int CompDeflatedThresh=90; // Threshold for compressor pressure when deflated
   int PressUpperThresh=160; // Threshold for upper pressure
   int PressLowerThresh=0; // Threshold for lower pressure
   int HeightUpperThresh=45; // Threshold for upper height
   int HeightLowerThresh=12; // Threshold for lower height
   int VbatUpperThresh=16; // Threshold for upper battery voltage
   int VbatLowerThresh=12.2; // Threshold for lower battery voltage
   int AngleUpperThresh=20; // Threshold for angle
   int PLowerThresh=90; // Threshold for deflation pressure

#ifdef useNEXTION
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

#endif

// Readed measures inside the Van (in mm) (date: 20210528)
// Maximum values: (TOF1:front  ; TOF2: rear sensor)
// TOF1=260  TOF2=310
// Values down front, up rear:
// TOF1=130  TOF2=350
// Values up front, down rear:
// TOF1=300  TOF2=170
// Values down front, down rear:
// TOF1=160  TOF2=200

// Pressure desired values (in centimeters) for each mode
int desPParkFront=80;
int desPParkRear=80;
int desPRoadFront=120;
int desPRoadRear=120;
int desPTrailFront=150;
int desPTrailRear=150;
int desPFront=100;
int desPRear=100;

/*  
   // Values chosen at 20210625
   // Height desired values (in centimeters) for each mode
   int desHParkFront=24;
   int desHParkRear=25;
   int desHRoadFront=26;
   int desHRoadRear=27;
   int desHTrailFront=30;
   int desHTrailRear=34;
*/
   
   /// VALUES FOR TESTING /////
   // Height desired values (in centimeters) for each mode
   int desHParkFront=23;
   int desHParkRear=23;
   int desHRoadFront=27;
   int desHRoadRear=27;
   int desHTrailFront=31;
   int desHTrailRear=31;
   
   int desHFront=23; // current desired value
   int desHRear=23; // current desired value





    ///////////////////////////////////////
    //////// Methods  //////////////////
    ///////////////////////////////////////

    void init();
    void readVbat();
    void readandcheckAll_Nsamples(int);
    void readAngle_onesample();
    void readPressures_onesample();
    void readPressures_Nsamples(int);
    void readAngle_Nsamples(int);
    void readAngle_Nsamples_discard(int);
    void readHeights_onesample();
    void readHeights_Nsamples(int);
    void readHeights_Nsamples_discard(int);
    void readTouchbutton();
    void checkVbat();
    void checkPComp();
    void checkHeights();
    void checkAngle();
    void getStatus();
    void updateNextionValues();
    void turnAllRelaysOFF();
    void analogbuttonpressed();
    

 private:
    int _buttonPin;
    int _switchPin;
    int _VbatPin;
    int _CompPin;
    int _rearPressurePin;
    int _frontPressurePin;
    int _rearUpRelayPin;
    int _rearDownRelayPin;
    int _frontUpRelayPin;
    int _frontDownRelayPin;
    char buff[50];

    void TOFSensorRead(unsigned char*, int);
    

};

#endif  // end if no defined JackVanControl_h
