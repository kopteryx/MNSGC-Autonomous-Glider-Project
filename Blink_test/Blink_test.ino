//Author: Billy Straub

#include <Servo.h>           //Servo library allows for servo use



#define ServoPin 9           //Defines the term "ServoPin" as pin 9 (Servo pins must be attached to PWM ports)
#define PotentiometerPin A0  //Defines the term "PotentiometerPin" as pin A0

Servo myservo;               //Defines "myservo" as a servo object

int PotentiometerValue;      //Creates an integer data type for "PotentiometerValue"



void setup() {
  
  myservo.attach(ServoPin);  //Attaches the servo to whatever pin "ServoPin" is defined as
  
}



void loop() {

  PotentiometerValue = analogRead(PotentiometerPin);              //The "PotentiometerValue" integer will equal what the "PotentiometerPin" reads
  PotentiometerValue = map(PotentiometerValue, 0, 1023, 0, 180);  //The "PotentiometerValue" originally reads from 0 to 1023 (b/c analog device), but the map function allows to the bounds to be changed to 0 to 180 
  myservo.write(PotentiometerValue);                              //The servo will move between 0 and 180 degrees based on the "PotentiometerValue"

}