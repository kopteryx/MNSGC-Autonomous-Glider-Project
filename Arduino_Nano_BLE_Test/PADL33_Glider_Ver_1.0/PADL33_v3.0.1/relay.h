#define RELAY_PIN  7

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

