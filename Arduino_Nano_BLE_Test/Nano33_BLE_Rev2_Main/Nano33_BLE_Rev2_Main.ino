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
#define RELAY_PIN  2
#include <string.h>

OpenLog myLog;
RTC_DS3231 rtc;
char t[32];

class OneWaySwitch {
private:
    bool reached_desired_altitude; //Boolean to determine whether the desired altitude was reached or not
public:
    OneWaySwitch() : reached_desired_altitude(false) {}
    bool turnOn() {
        reached_desired_altitude = true;
        
        for (int i = 0; i < 2; i++) {
          digitalWrite(RELAY_PIN, HIGH);
          //Serial.println("Relay is ON");
          // myLog.println("Relay is ON");
          delay(12000);
          digitalWrite(RELAY_PIN, LOW);
          //Serial.println("Relay is OFF");
          // myLog.println("Relay is OFF");
          delay(10000);
        }
        return true;
    }
    bool getState() const {
        return reached_desired_altitude;
    }
};

OneWaySwitch switch1;
float desired_altitude = 24700; //Desired altitude + sea-level @ Montgomery MN: 81,000 ft = 24700 m
bool isCut;
String data = "";

void setup() {
  Wire.begin();
  myLog.begin();
  isCut = false;
  
  //Serial.begin(9600);
  //while (!Serial);

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
  //rtc.adjust(DateTime(2024, 7, 23, 13, 16, 0));

  // initialize digital pin as an output.
  pinMode(RELAY_PIN, OUTPUT);

  String fileName = "Nano33_BLE_Rev2_Data.csv";

  myLog.create(fileName);
  myLog.append(fileName);
  myLog.syncFile();
  
  // myLog.println();
  // myLog.println();
  myLog.print("Started Logging New Data!\n");
  myLog.print("Date/Time,Pressure(kPa),Temperature(C),Humidity(%), Altitude(m),Relay(on/off)\n");
  //Serial.println("SD is sucessufully logging!");
  // myLog.println();
}

void loop() {
    DateTime now = rtc.now();
    sprintf(t, "%02d:%02d:%02d %02d/%02d/%02d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());  
    // myLog.print(F("Date/Time: "));
    // myLog.println(t);
    data += t;
    data += ",";

    float pressure = BARO.readPressure();

    // myLog.print("Pressure    = ");
    // myLog.print(pressure);
    // myLog.println(" kPa");
    data += String(pressure);
    data += ",";

    float temperature = BARO.readTemperature();
    float humidity    = HS300x.readHumidity();

    // myLog.print("Humidity    = ");
    // myLog.print(humidity);
    // myLog.println(" %");
    data += String(humidity);
    data += ",";

    // myLog.print("Temperature = ");
    // myLog.print(temperature);
    // myLog.println(" C");
    data += String(temperature);
    data += ",";

    float altitude = 44330 * ( 1 - pow(pressure/101.325, 1/5.255) );
    
    // myLog.print("Altitude according to kPa = ");
    // myLog.print(altitude);
    // myLog.println(" m");
    // myLog.println();
    data += String(altitude);
    data += ",";

    /*
    Serial.print("Altitude according to kPa = ");
    Serial.print(altitude);
    Serial.println(" m");
    Serial.println();
    */

    //Checks if current altitude has reached the desired altitiude, then turns on a one way switch to active the release mechanism
    if (altitude >= desired_altitude && !switch1.getState()) {
        isCut = switch1.turnOn();
    }

    data += String(isCut);
    data += ",";
    data += "\n";

    myLog.print(data);

    data = "";
    isCut = false;

    delay(1000);
  } 