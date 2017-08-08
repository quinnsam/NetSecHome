/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "<Blynk API>";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "<SSID>";
char pass[] = "<WIFI PASSWORD>";

// Select your pin with physical button
const int zone1Pin = D1;
const int zone2Pin = D2;
const int zone3Pin = D3;
const int alarmPin = D4;


WidgetLED led1(V1);
WidgetLED led2(V2);
WidgetLED led3(V3);

SimpleTimer timer;

boolean Zone1State = false;
boolean Zone2State = false;
boolean Zone3State = false;
boolean ArmStatus  = true;

void notify(int zone)
{
    if (ArmStatus) {
        if (zone == 1) {
            Blynk.notify("The Front Door has Been Opened");
        } else if (zone == 2) {
            Blynk.notify("The Sliding Door has Been Opened");
        } else if (zone == 3) {
            Blynk.notify("Motion Sensor has Been Triggered");
        }
    }
}

void checkZones()
{
  // Read Zone 1
  boolean zone1Triggered = (digitalRead(zone1Pin) == LOW);

  // If state has changed...
  if (zone1Triggered != zone1State) {
    if (zone1Triggered) {
      led1.on();
    } else {
      led1.off();
      notify(1);
    }
    zone1State = zone1Triggered;
  }

  // Read Zone 2
  boolean zone2Triggered = (digitalRead(zone2Pin) == LOW);

  // If state has changed...
  if (zone2Triggered != zone2State) {
    if (zone2Triggered) {
      led2.on();
    } else {
      led2.off();
      notify(2);
    }
    zone2State = zone2Triggered;
  }

  // Read Zone 3
  boolean zone3Triggered = (digitalRead(zone3Pin) == LOW);

  // If state has changed...
  if (zone3Triggered != zone3State) {
    if (zone3Triggered) {
      led3.on();
    } else {
      led3.off();
      notify(3);
    }
    zone3State = zone3Triggered;
  }

}

// Monitor the ARM status.
BLYNK_WRITE(V0)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V0 to a variable
  if (pinValue == 1) {
      ArmStatus = true;
  }
  else if (pinValue == 0) {
      ArmStatus = false;
  }
}

// Alarm.
BLYNK_WRITE(V4)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V4 to a variable
  if (pinValue == 1) {
      digitalWrite(alarmPin, HIGH);
  }
  else if (pinValue == 0) {
      digitalWrite(alarmPin, LOW);
  }
}

void setup()
{
  // Debug console
  Serial.begin(9600);

  Blynk.begin(auth, ssid, pass);

  // Setup physical zone pins (active low)
  pinMode(zone1Pin, INPUT_PULLUP);
  pinMode(zone2Pin, INPUT_PULLUP);
  pinMode(zone3Pin, INPUT_PULLUP);

  pinMode(alarmPin, OUTPUT);
  digitalWrite(alarmPin, LOW);

  timer.setInterval(500L, checkZones);
}

void loop()
{
  Blynk.run();
  timer.run();
}
