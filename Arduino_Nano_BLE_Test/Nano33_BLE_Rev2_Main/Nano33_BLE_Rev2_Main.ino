/*
  EDIT TO SPECIFY UAV PROJECT CODE

  The circuit:
  - Arduino Nano 33 BLE Sense
*/

#include <Arduino_LPS22HB.h>
#include <Arduino_HS300x.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#define RELAY_PIN  2  // The Arduino Nano pin connected to the IN pin of relay module

OpenLog myLog;
RTC_DS3231 rtc;
char t[32];

class OneWaySwitch {
private:
    bool reached_desired_altitude; //Boolean to determine whether the desired altitude was reached or not
public:
    OneWaySwitch() : reached_desired_altitude(false) {}
    void turnOn() {
        reached_desired_altitude = true;
        
        for (int i = 0; i < 2; i++) {
          digitalWrite(RELAY_PIN, HIGH);
          Serial.println("Relay is ON");
          myLog.println("Relay is ON");
          delay(12000);
          digitalWrite(RELAY_PIN, LOW);
          Serial.println("Relay is OFF");
          myLog.println("Relay is OFF");
          delay(10000);
        }

        myLog.println();
    }
    bool getState() const {
        return reached_desired_altitude;
    }
};

OneWaySwitch switch1;    // CHANGE TO CORRECT ALTITUDE + SEA LEVEL BEFORE FLIGHT
float desired_altitude = 282.5; //Desired altitude in meters

void setup() {
  Wire.begin();
  myLog.begin();
  
  Serial.begin(9600);
  while (!Serial);

  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }

  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  //rtc.adjust(DateTime(2024, 7, 15, 11, 34, 0));

  // initialize digital pin as an output.
  pinMode(RELAY_PIN, OUTPUT);

  myLog.append("Nano33_BLE_Rev2_Data.txt");
  myLog.syncFile();
  
  myLog.println();
  myLog.println();
  myLog.println();
  //Serial.println("SD is sucessufully logging!");
  myLog.println("Started Logging New Data!");
  myLog.println();
}

void loop() {
    DateTime now = rtc.now();
    sprintf(t, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());  
    myLog.print(F("Date/Time: "));
    myLog.println(t);

    float pressure = BARO.readPressure();

    myLog.print("Pressure    = ");
    myLog.print(pressure);
    myLog.println(" kPa");

    float temperature = BARO.readTemperature();
    float humidity    = HS300x.readHumidity();

    myLog.print("Humidity    = ");
    myLog.print(humidity);
    myLog.println(" %");

    myLog.print("Temperature = ");
    myLog.print(temperature);
    myLog.println(" C");

    float altitude = 44330 * ( 1 - pow(pressure/101.325, 1/5.255) );
    
    myLog.print("Altitude according to kPa = ");
    myLog.print(altitude);
    myLog.println(" m");
    myLog.println();

    //Checks if current altitude has reached the desired altitiude, then turns on a one way switch to active the release mechanism
    if (altitude >= desired_altitude && !switch1.getState()) {
        switch1.turnOn();
    }

    // wait 1 second to print again
    delay(1000);
  } 