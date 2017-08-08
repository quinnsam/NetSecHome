/******************************************************************************
 * This program automates a door for keyless unlocking                         *
 * Developed by: Sam Quinn, Chauncey Yan, Ashley Greenacre, and Chris Harper.  *
 * 05/13/2014                                                                  *
 ******************************************************************************/
#include <Wire.h>
#include <Time.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SimpleTimer.h>

// WIFI Variables
const char* ssid     = <ssid>;
const char* password = <password>;

// MQTT Variables
const char* mqtt_server = <mqttserver>;
const char* mqtt_user = <username>;
const char* mqtt_pass = <password>;

const char* mqtt_topic_door_out  = <out>;
const char* mqtt_topic_door_in   = <in>;
const char* mqtt_topic_netsec_in = <out>;

//Servo Controller i2c
Adafruit_PWMServoDriver door = Adafruit_PWMServoDriver(0x40);

// MQTT Instance
WiFiClient espClient;
PubSubClient client(espClient);
char msg[50];

// Possible sensor addresses (suffix correspond to DIP switch positions)
#define SENSOR_ADDR_OFF_OFF  (0x26)
#define SENSOR_ADDR_OFF_ON   (0x22)
#define SENSOR_ADDR_ON_OFF   (0x24)
#define SENSOR_ADDR_ON_ON    (0x20)

// Lock angle definitions
#define LOCK        600
#define UNLOCK      1990

// Time Definitions
#define PRX_WAIT   10000   // Time to wait before locking after proximity trigger
#define SYS_WAIT  2     // Short pasue to allow system to catch up  
#define RUN_WAIT  5      // Time to wait before starting loop again
#define CAL_WAIT  1500    // Time to wait for the calibrator
#define ADJ_WAIT  1000    // Time to wait for the Servo to adjust
#define DSR_WAIT  1500     // Delay before locking after the door sensor is triggered
#define AFT_WAIT  500    // Time to wait to allow door to complete its task

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  500 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2500 // this is the 'maximum' pulse length count (out of 4096)

// Functions declarations
extern int ReadByte(uint8_t addr, uint8_t reg, uint8_t *data);
extern void WriteByte(uint8_t addr, uint8_t reg, byte data);
extern int lock_status();
extern int lock(int lock_pos);
extern void calibrate();
extern void serial_monitor();
extern void prox_monitor();
extern void door_monitor();
extern void setup_wifi();
extern void callback(char* topic, byte* payload, unsigned int length);
extern void reconnect();
extern void publishData();

// Set the Proximity sensor address here
const uint8_t sensorAddr = SENSOR_ADDR_OFF_OFF;

// Pins
int led_pin = BUILTIN_LED;       // LED connected to digital pin 13
int pot_pin = A0;     // analog pin used to connect the potentiometer

// Global Varibles
int pot_val = -1;       // variable to read the value from the analog pin 
int pot_lock = 0;
int pot_unlock = 0;
int input;
int lockdown = 0;
volatile int door_status = 0;
volatile int lock_next = 0;
SimpleTimer timer;


//global counter
time_t relock_counter;
int lockdown_counter = 0;
int door_sensor = -1;

// One-time setup
void setup()
{
    // Start the serial port for output
    Serial.begin(9600);

    // Start WIFI and MQTT
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    // Pin to connet to the pi
    pinMode(led_pin, OUTPUT);      // sets the digital pin as output

    // Join the I2C bus as master
    Wire.begin();

    // Adrress for the proximity sensor 
    WriteByte(sensorAddr, 0x3, 0xFE);

    door.begin();
    door.setPWMFreq(330);  // Analog servos run at ~60 Hz updates
    delay(100);

    timer.setInterval(500, publishData);
}


// Main program loop
void loop() {
    int count = 0;
    // Check for serial input
    serial_monitor();
    // Check for Proximity trigger
    
    //Begin proximity monitoring
    // 1. Connect one end of the cable into either Molex connectors on the sensor
    //Connect the other end of the cable to the Arduino board:
    //RED: 5V
    //WHITE:  I2C SDA (pin A4 on Uno; pin 20 on Mega)
    //BLACK: GND
    //GREY: I2C SCL (pin A5 on Uno; pin 21 on Mega)
    //Set the DIP switch on the sensor to set the sensor address (check back of sensor for possible addresses)
    // Varible to store proximity data in
    
    uint8_t val;

    // Get the value from the sensor
    if (ReadByte(sensorAddr, 0x0, &val) == 0) {
        /* The second LSB indicates if something was not detected, i.e.,
           LO = object detected, HI = nothing detected */
        if (val & 0x2) {
            //Serial.println("Nothing detected");
            delay(SYS_WAIT);
        } else {
            Serial.println("Proximity Sensor: Object detected");

            if (lock(0) != 0) {
                Serial.println("ERROR: Could not execute command UNLOCK");
            }
            delay(PRX_WAIT);
            if (lock(1) != 1) {
                Serial.println("ERROR: Could not execute command LOCK");
            }

            delay(SYS_WAIT);
        }
    } else {
        Serial.println("Failed to read from sensor");
    }
    
    // Check wheater the door is locked or not and auto lock if unlocked.
    door_monitor();
    // Run again in 0.5 s (500 ms)
    delay(RUN_WAIT);

    if (!client.connected()) {
      reconnect();
    } else if (count > 5000){
      publishData();
      count = 0;
    }
    count++;
    client.loop();
    if (lock_next > 2) {
      lock(1);
      Serial.print("Lock Next: ");
    Serial.println(lock_next);
    }

    timer.run();
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonBuffer<200> jsonBuffer;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, mqtt_topic_netsec_in) == 0) {
    Serial.println("in netsec");
    
    JsonObject& root = jsonBuffer.parseObject(payload);
    if(!root.success()) {
      Serial.println("JSON Failed");
      return;
    }
    if (strcmp(root["zone1"], "ON") == 0) {
      Serial.println("Front door closed");
      door_status = 1;
      if (lock_next) { lock_next++; }
    } else {
      Serial.println("Front door open");
      door_status = 0;
    }
  } else {

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '0') {
    lock(0);
  } else if ((char)payload[0] == '1') {
    lock(1);
  } else if ((char)payload[0] == '3') {
    calibrate();
  } else if ((char)payload[0] == '4') {
    publishData();
  }
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe(mqtt_topic_door_in);
      client.subscribe(mqtt_topic_netsec_in);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// function called to publish the door status
void publishData() {
  int l_stat = lock_status();
  if (l_stat == 1) {
    client.publish(mqtt_topic_door_out, "1", true);
  } else if (l_stat == 0) {
    client.publish(mqtt_topic_door_out, "0", true);
  } else {
    client.publish(mqtt_topic_door_out, "-1", true);
  }
}

void door_monitor() {
    // check if the door is unlocked. 
    // lock it after about 20 (0*0.05) seconds 
    // if no more interaction detected.
    if (lock_status() != 1){
        if ( lockdown ) {
            lockdown_counter++;
            if (lockdown_counter == 1)
                Serial.println("UNLOCKED");
        }
        if ( now() >= relock_counter + 120 ){
            lock(1);
        }
    } else {
        relock_counter = now();
        lockdown_counter = 0;
    }
}


void serial_monitor() {
    int stat;
    //Beginig Serial monitoring
    if (Serial.available() > 0) {
        input = Serial.read();
        if (input == '0') {
            if (lock(0) != 0) {
                Serial.println("ERROR: Could not execute command UNLOCK");
            }
        } else if (input == '1') {
            if (lock(1) != 1) {
                Serial.println("ERROR: Could not execute command LOCK");
            }
        } else if (input == '2') {
            stat = lock_status();
            if (stat == 1) {
                Serial.println("LOCKED");
                publishData();
            } else if (stat == 0){
                Serial.println("UNLOCKED");
                publishData();
            } else {
                Serial.print("ERROR:");
                Serial.println(analogRead(pot_pin));
            }
        } else if (input == '3') {
            calibrate();
        } else if (input == '9') {
            lockdown = !lockdown;
            if ( lockdown ) {
                Serial.println("ATTN: LOCKDOWN ENABLED");
            } else {
                Serial.println("ATTN: LOCKDOWN DISABLED");
            }

        } else {
            Serial.print("ERROR: Unreconnized command: ");
            char out = input;
            Serial.print("(");
            Serial.print(out);
            Serial.println(")");
        }
    }
}


// Read a byte on the i2c interface
int ReadByte(uint8_t addr, uint8_t reg, uint8_t *data) {
    // Do an i2c write to set the register that we want to read from
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    // Read a byte from the device
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
        *data = Wire.read();
    } else {
        // Read nothing back
        return -1;
    }

    return 0;
}

// Write a byte on the i2c interface
void WriteByte(uint8_t addr, uint8_t reg, byte data) {
    // Begin the write sequence
    Wire.beginTransmission(addr);

    // First byte is to set the register pointer
    Wire.write(reg);

    // Write the data byte
    Wire.write(data);

    // End the write sequence; bytes are actually transmitted now
    Wire.endTransmission();
}

void calibrate () {
    //unlock the door to read the potvalue 
    door.setPWM(0,0,UNLOCK);
    delay(CAL_WAIT);
    door.setPWM(0,0,0);
    delay(ADJ_WAIT);
    // read the value of the potentiometer
    pot_unlock = analogRead(pot_pin); 
    // print out the value to the serial monitor
    Serial.print("Defined unlock: ");
    Serial.println(pot_unlock);

    //lock the door to read the potvalue 
    door.setPWM(0,0,LOCK);
    delay(CAL_WAIT);
    // read the value of the potentiometer
    door.setPWM(0,0,0);
    delay(ADJ_WAIT);
    pot_lock = analogRead(pot_pin); 
    // print out the value to the serial monitor
    Serial.print("Defined lock: ");
    Serial.println(pot_lock);
}

/******************************************************************************
 * Determines the current state of the door
 *
 * Tasks:
 * 1.)   Read analog data from the servos internal potentiometer
 * 2.)   Map the potentiometer data to an angle
 * 3.)   If angle is close to the defined LOCK value return 1
 * 4.)   If angle is close to the defined UNLOCK value return 0
 * 5.)   If the lock is in an indeterminate state then return -1
 *
 ******************************************************************************/
int lock_status() {
    int rv = -1;

    pot_val = analogRead(pot_pin); // read the value of the potentiometer

    //print_info();

    if(pot_val >= (pot_lock -15) && pot_val <= (pot_lock + 15)){
      digitalWrite(led_pin,LOW);
      return 1; // Locked
    } else if(pot_val >= (pot_unlock -15) && pot_val <= (pot_unlock + 15)) {
      digitalWrite(led_pin,HIGH);
      return 0; // Unlocked
    } else {
      return -1; // Error
    }
}

int door_pos() {
  
}


/******************************************************************************
 * Will either lock (1) or unlock (0) the door 
 * 
 * Tasks:
 * 1.)   Attach to the Servo motor
 * 2.)   Read the curent position of the lock
 * 3.)   If the door is already in its desired location do nothing
 * 4.)   If the door is not in the desired location then set the servo angle
 * 5.)   Move the servo to the desired location
 * 6.)   Detach the servo to allow manual locking and unlocking.
 *
 ******************************************************************************/
int lock(int lock_pos) {

    int l_status = lock_status();
    int angle = LOCK;
    time_t door_open_time;


    if (lock_pos == 1) {
        Serial.println("----LOCKING----");
    } else if (lock_pos == 0) {
        Serial.println("----UNLOCKING----");
    } else {
        Serial.print("Unreconized command for lock():");
        Serial.println(lock_pos);
    }

    // Read the position of the lock currently
    if (l_status == lock_pos) {
        Serial.println("ALREADY ins desired state.");
        return lock_pos;
    } else {
        //print_info();
        if (lock_pos == 1) {
            angle = LOCK;
        } else if (lock_pos == 0) {
            angle = UNLOCK;
        }
    }

    // set the servo position  
    if (angle == LOCK) {
      if (lock_next > 2) {
        door.setPWM(0,0,LOCK);
        lock_next = 0;
      } else if (lock_next == 0) {
        lock_next = 1;
        return 1;
      }
    } else {
      // No need to wait for the door to close to unlock
        door.setPWM(0,0,UNLOCK);
    }
    delay(AFT_WAIT);

    // Detach servo so manual over ride can take place
    door.setPWM(0,0,0);

    return lock_status();
}

