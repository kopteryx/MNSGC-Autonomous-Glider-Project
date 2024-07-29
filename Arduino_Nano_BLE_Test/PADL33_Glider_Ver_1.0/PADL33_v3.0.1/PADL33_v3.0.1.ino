/************************************************
      MnSGC Ballooning/Rocketry PADL-33 Flight Code
      Created by: Ashton Posey

      Modification Date: 7/3/24
************************************************/
//Purpose: Code for the PADL-33 Flight Computer
#define Version "Version 3.0"

#include <ReefwingLPS22HB.h>
#include <Arduino_HS300x.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include "DFRobot_BMM150.h"
#include <Arduino_APDS9960.h>
#include <SdFat.h>
#include <Wire.h>
#include <RTClib.h>
//#include <SafeString.h>
#include <math.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <SparkFun_KX13X.h> //http://librarymanager/All#SparkFun_KX13X
#include <SFE_MicroOLED.h>



#include "OLED.h"
#include "Thermistor.h"
#include "LED.h"
#include "variables.h"

// VARIABLES TO EDIT FOR CONFIGURATION
#define GPS_FREQUENCY 10
#define GPS_DATA_DELAY 100 // milliseconds
#define DATA_DELAY 100 // milliseconds : can be different than GPS_DATA_DELAY if you want the gps to update slower than the rest of the sensors
#define GPS_BAUD 38400 // 38400 for M9N : 9600 for M8N

bool ecefEnabled = true; // bool to enable turing off ECEF for debugging
bool rtkEnabled = false; // bool to enable turing off RTK for debugging -> Must be using D9S Corrections Reciever
bool gpsI2C = false;      // bool to enable I2C when true, UART when false
bool usingBuzzer = false; // if this is false, the buzzer will never make a noise. 
bool usingOLED = true; // if false, OLED screen won't be used and setup will be faster.
// END VARIABLES TO EDIT FOR CONFIGURATION

String header = "hh:mm:ss,FltTimer,T(s),T(ms),Hz,T2,T3,T4,T5,T6,totT,5v,VIN(V),HtrS,extT(F) or ADC,extT(C),intT(F),intT(C),Fix Type,RTK,PVT,Sats,Date,Time,Lat,Lon,Alt(Ft),Alt(M),HorizAccuracy(MM),VertAccuracy(MM),VertVel(Ft/S),VertVel(M/S),ECEFstat,ECEFX(M),ECEFY(M),ECEFZ(M),NedVelNorth(M/S),NedVelEast(M/S),NedVelDown(M/S),GndSpd(M/S),Head(Deg),PDOP,kPa,ATM,PSI,C,F,Ft,M,VV(Ft),VV(M),G(y),G(x),G(z),Deg/S(x),Deg/S(y),Deg/S(z),uT(x),uT(y),uT(z),kx mG(y),mG(x),mG(z),GPS I2C?,Atomic clock,Cut?," + String(Version);

void setup() { //////////////////////////////////////////// SETUP ////////////////////////////////////////////
    systemSetUp();
    if (usingBuzzer){
        startUpJingle();
    }
} ///////////////////////////////////////////////////////// SETUP ////////////////////////////////////////////

void loop() { ///////////////////////////////////////////// LOOP /////////////////////////////////////////////

    if(millis() - timer >= DATA_DELAY){
        timer = millis();
        updateData();
    }

    // if (rtkEnabled){
    //   if (millis() - timer <= DATA_DELAY - 7){
    //       sparkFunGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
    //       sparkFunGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.
    //   }
    // }

} ///////////////////////////////////////////////////////// LOOP /////////////////////////////////////////////

///////// Functions ////////////
// void updateData();

void updateData(){
    systemUpdate();

    data = flightTimer;
    data += ",";
    data += flightTimerString;
    data += ",";
    data += String(timerSec);
    data += ",";
    data += String(timer);
    data += ",";
    data += String(frequencyHz);
    data += ",";
    data += String(timer2);
    data += ",";
    data += String(timer3);
    data += ",";
    data += String(timer4);
    data += ",";
    data += String(timer5);
    data += ",";
    data += String(timer6);
    data += ",";
    data += String(timerTotal);
    data += ",";
    data += String(voltage_5v);
    data += ",";
    data += String(voltage_3v7);
    data += ",";
    data += String(heaterStatus);
    data += ",";

    if(thermExtOrADC){
        data += String(ThermistorExt.getTempF());
        data += ",";
        data += String(ThermistorExt.getTempC());
        data += ",";
    }
    else{
        data += String(photoresistorADCValue);
        data += ",";
        data += "0";
        data += ",";
    }

    data += String(ThermistorInt.getTempF());
    data += ",";
    data += String(ThermistorInt.getTempC());
    data += ",";
    data += fixTypeGPS;
    data += ",";
    data += fixTypeRTK;
    data += ",";
    data += String(PVTstatus);
    data += ",";
    data += String(SIV);
    data += ",";
    data += String(gpsMonth);
    data += "/";
    data += String(gpsDay);
    data += "/";
    data += String(gpsYear);
    data += ",";
    data += String(gpsHour);
    data += ":";
    data += String(gpsMinute);
    data += ":";
    data += String(gpsSecond);
    data += ".";

    if (gpsMillisecond < 10) {
        data += "00";
        data += String(gpsMillisecond);
        data += ",";
    }
    else if (gpsMillisecond < 100) {
        data += "0";
        data += String(gpsMillisecond);
        data += ",";
    }
    else{
        data += String(gpsMillisecond); 
        data += ",";
    }

    // data += lat_int;
    // data += ".";
    // data += lat_frac;
    // data += ",";
    // data += lon_int;
    // data += ".";
    // data += lon_frac;
    // data += ",";
    // data += f_ellipsoid*1000;
    // data += ",";
    // data += f_msl*1000;
    // data += ",";
    // data += f_accuracy*1000;
    // data += ",";

    char paddedNumber[8]; // Buffer to hold the padded number (7 digits + null terminator)
    data += String(gpsLatInt);
    data += ".";
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLatDec);
    data += String(paddedNumber); // Pad the number with zeros up to 7 digits
    // data += gpsLatDec;
    data += ",";

    data += String(gpsLonInt); 
    data += ".";
    // Format the number with padded zeros using sprintf()
    sprintf(paddedNumber, "%07ld", gpsLonDec);
    data += String(paddedNumber); // Pad the number with zeros up to 7 digits
    // data += gpsLonDec;
    data += ",";

    // data += gpsLatInt;
    // data += ".";
    // data += gpsLatDec;
    // data += ",";
    // data += gpsLonInt;
    // data += ".";
    // data += gpsLonDec;
    // data += ",";
    data += String(gpsAltFt);
    data += ",";
    data += String(gpsAltM);
    data += ",";
    data += String(gpsHorizAcc);
    data += ",";
    data += String(gpsVertAcc);
    data += ",";
    data += String(gpsVertVelFt);
    data += ",";
    data += String(gpsVertVelM);
    data += ",";
    data += String(ecefStatus);
    data += ",";
    data += String(ecefX);
    data += ",";
    data += String(ecefY); 
    data += ",";
    data += String(ecefZ);
    data += ","; 
    data += String(velocityNED[0]);
    data += ",";
    data += String(velocityNED[1]); 
    data += ",";
    data += String(velocityNED[2]);
    data += ","; 
    data += String(gpsGndSpeed);
    data += ",";
    data += String(gpsHeading);
    data += ",";
    data += String(gpsPDOP);
    data += ",";
    data += String(pressureSensor[0]);
    data += ",";
    data += String(pressureSensor[1]);
    data += ",";
    data += String(pressureSensor[2]);
    data += ",";
    data += String(pressureSensor[3]);
    data += ",";
    data += String(pressureSensor[4]);
    data += ",";
    data += String(pressureSensor[5]);
    data += ",";
    data += String(pressureSensor[6]);
    data += ",";
    data += String(vertVelFt);
    data += ",";
    data += String(vertVelM);
    data += ",";
    data += String(imu.data.accelX);
    data += ",";
    data += String(imu.data.accelY);
    data += ",";
    data += String(imu.data.accelZ);
    data += ",";
    data += String(imu.data.gyroX);
    data += ",";
    data += String(imu.data.gyroY);
    data += ",";
    data += String(imu.data.gyroZ);
    data += ",";
    data += String(magData.x);
    data += ",";
    data += String(magData.y);
    data += ",";
    data += String(magData.z);
    data += ",";
    data += String(kxData[0]);
    data += ",";
    data += String(kxData[1]);
    data += ",";
    data += String(kxData[2]);
    data += ",";
    data += String(gpsI2C);
    data += "\n";

    Serial.println(data);
    
    timer5 = millis() - timer7; ///////////// Timer 6 ///////////// 
    

    timer6 = millis(); ///////////// Timer 5 ///////////// 

    if (usingOLED){
      String strOLED, strOLED2;

      strOLED = String(gpsLat,4) + "\n" + String(gpsLon,4) + "\n" + String(gpsAltFt, 2) + "ft\n";

      strOLED += String(SIV);
      strOLED2 = String(SIV);
      for(int i=strOLED2.length(); i<3; i++){
        strOLED += " ";
      }
      strOLED += "S " + String(frequencyHz, 1) + "\nI:" + String(int(ThermistorInt.getTempF())) + " E:" + String(int(ThermistorExt.getTempF())) + "\n" + String(pressureSensor[5]) + " Ft";
      OLED.update(strOLED);
    }

    dataAdded += data;
    if (dataCounter == 3){
        if (SD.exists(filename)){
            datalog.print(dataAdded);
            datalog.flush();
        }
        else{
            if (usingBuzzer) tone(TONE_PIN, 400);
            Serial.println("NO SD");
        }
        // datalog.print(dataAdded);
        // datalog.flush();
        dataAdded = "";
    }
    dataCounter++;
    if (dataCounter == 4) dataCounter = 0;

    if (usingXBee)  XBee.println(String(pressureSensor[5]));

    timer6 = millis() - timer6; /////////////////////////

    timerTotal = timer2 + timer3 + timer4 + timer5 + timer6;

} /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

