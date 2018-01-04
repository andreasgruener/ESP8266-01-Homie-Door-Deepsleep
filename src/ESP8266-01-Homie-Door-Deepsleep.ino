#include <Homie.h>
#include <DHT.h>


#define DHTPIN 2 // D4
#define DHTTYPE DHT22    // Es handelt sich um den DHT11 Sensor
#define DOOR_PIN 12 // D6 
#define POWER_PIN 14 // D5

#define FW_NAME "deepsleep-werkstatt"
#define FW_VERSION "2.0.0"


/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" FW_NAME "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" FW_VERSION "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */


// 5ccf7fa47b9e HomeTemperature3 # GRETA
// mosquitto_pub -h 10.0.2.2 -t "homie/5ccf7fa47b9e/light/on/set" -m "true"
// mosquitto_sub -h 10.0.2.2 -t "homie/5ccf7fa47b9e/humidity/percent"
// mosquitto_pub -h 10.0.2.2 -t "homie/5ccf7fa47b9e/temperature/delay/set" -m "300"
// mosquitto_sub -h 10.0.2.2 -t "homie/5ccf7fa47b9e/temperature/degrees"

// mosquitto_pub -r -h 10.0.2.2 -t "homie/2c3ae8225e86/deepsleep/sleeptime/set" -m "600"
// mosquitto_pub -r -h 10.0.2.2 -t "homie/2c3ae8225e86/deepsleep/enabled/set" -m "true"

// comment
const int PIN_RELAY = LED_BUILTIN;
const int TEMPERATURE_INTERVAL = 5;

int temperature_interval = TEMPERATURE_INTERVAL;
unsigned long lastTemperatureSent = 0;
unsigned long lastRetry = 0;
int maxRetries = 5;
int retries = 0;


/* DEEPSLEEP */
const int DEFAULT_DEEP_SLEEP_TIME = 600;
int deepsleeptime = DEFAULT_DEEP_SLEEP_TIME;
bool deepSleepEnabled = true;

/* DOOR STUFF */
unsigned long lastPowerDoorCheckTime = 0;
int door_current_state = LOW;      // the current state of the output pin

/** POWER CONNECTED //  LIGHTS ON 
*/
unsigned long lastPowerDoor = 0;
int power_current_state = LOW;   

// battery
double battery_voltage;

HomieNode lightNode("light", "switch");
HomieNode temperatureNode("temperature", "temperature");
HomieNode humidityNode("humidity", "humidity");
HomieNode doorNode("door", "door");
HomieNode powerNode("power", "power");
HomieNode batteryNode("battery", "battery");
HomieNode deepsleepNode("deepsleep", "deepsleep");

// Temperature
DHT dht(DHTPIN, DHTTYPE); //Der Sensor wird ab jetzt mit DHT angesprochen

// if you nee getVCC() turn this on
ADC_MODE(ADC_VCC);

void setupHandler() {
  temperatureNode.setProperty("unit").send("c");
  humidityNode.setProperty("unit").send("%");
  doorNode.advertise("door");
 
  powerNode.advertise("power");
  
  batteryNode.setProperty("unit").send("V");
  
  deepsleepNode.advertise("sleeptime").settable(deepsleepHandler);
  deepsleepNode.setProperty("sleeptime").send(String(deepsleeptime));

  deepsleepNode.advertise("enabled").settable(deepsleepEnabledHandler);
  deepsleepNode.setProperty("enabled").send(String(deepSleepEnabled));
}



bool delayHandler(const HomieRange& range, const String& value) { 
  
  Homie.getLogger() << "Got Delay " << value << endl;;
  int delay = value.toInt();
  temperature_interval = delay;
  temperatureNode.setProperty("delay").send(value);
  humidityNode.setProperty("delay").send(value);
  Homie.getLogger() << "Delayis " << delay << endl;

  return true;
}


bool lightOnHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  lightNode.setProperty("on").send(value);
  Homie.getLogger() << "Light is " << (on ? "on" : "off") << endl;

  return true;
}


/*
*
*/
bool deepsleepEnabledHandler(const HomieRange& range, const String& value) { 
  
  Homie.getLogger() << "MQTT Input :: Got Deepsleep Enabled " << value << endl;;
  if (value != "true" && value != "false") return false;

  bool enabled = (value == "true");
  deepSleepEnabled  = enabled;
  deepsleepNode.setProperty("enabled").send(value);

  Homie.getLogger() << "   Deepsleep Enabled value  " << deepSleepEnabled << endl;


  return true;
}

/*
NOt yet working - keeps forgetting...
*/
bool deepsleepHandler(const HomieRange& range, const String& value) { 
  
  Homie.getLogger() << "Got Deepsleep value (seconds) " << value << endl;;
  int delay = value.toInt();
  deepsleeptime  = delay;
  deepsleepNode.setProperty("sleeptime").send(value);

  Homie.getLogger() << "Deepsleep Delay Value  " << delay << endl;
  
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;
  
  pinMode(DOOR_PIN, INPUT);
  pinMode(POWER_PIN, INPUT);
  
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  Homie_setFirmware(FW_NAME, FW_VERSION);
  Homie_setBrand("mercatis-iot");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  
  lightNode.advertise("on").settable(lightOnHandler);
  lightNode.setProperty("on").send("false");
  
  temperatureNode.advertise("delay").settable(delayHandler);
  temperatureNode.setProperty("delay").send(String(temperature_interval));
 
  humidityNode.advertise("delay").settable(delayHandler);
  humidityNode.setProperty("delay").send(String(temperature_interval));
  
  Homie.onEvent(onHomieEvent);
  Homie.setup();
  dht.begin(); //DHT11 Sensor starten
  // sensors.begin(); //OneWire/Dallas
}

void onHomieEvent(const HomieEvent& event) {
  switch(event.type) {
    // case HomieEventType::MQTT_READY:
    //   Homie.getLogger() << "MQTT connected, preparing for deep sleep..." << endl;
    //   Homie.prepareToSleep();
    //   break;
    case HomieEventType::READY_TO_SLEEP:
      Homie.getLogger() << "## Ready to sleep. Initiate Deepsleep for " << deepsleeptime << " seconds" << endl;
      ESP.deepSleep(deepsleeptime * 1000 * 1000); // (microseconds)
      break;
  }
}


float getTemperature() {  
  float temperature = dht.readTemperature();
  return temperature;
}

float getHumidity() {
  float humidity = dht.readHumidity(); 
 return humidity;
}


void checkDHT11() {
  
  if (  ( retries < maxRetries && millis() - lastRetry >= 3000UL ) ||
        millis() - lastTemperatureSent >= temperature_interval * 1000UL ) {

    float temperature = 0;
    float humidity = 0;
    float pressure = 0;
    lastTemperatureSent =  millis();

    // successful the lasttime, new game
    if ( retries > maxRetries ) {
      retries = 0;
    } else if ( retries == maxRetries ) {
      Homie.getLogger() << "MAx retries reached - Sensor not OK - giving up .." << endl;
      temperatureNode.setProperty("sensor_ok").send("0");
      retries++;
      if ( deepSleepEnabled ) {
        Homie.prepareToSleep();
      }
      return;
    }

    // try getting temperature -- sometimes there is no answer from the sensor
    temperature = getTemperature();
    //Homie.getLogger() << "Temperature reading " << temperature << endl;
    if ( isnan(temperature) || temperature == 0 ) {
      retries++;
      Homie.getLogger() << "*** Temperature not available retrying " << retries << ". time " << " max Retries = " << lastRetry << "... " << millis() << endl;
      lastRetry = millis();
      temperatureNode.setProperty("sensor_ok").send("0");
    }
    else {

      Homie.getLogger() << "Temperature: " << temperature << " °C" << endl;
      temperatureNode.setProperty("degrees").send(String(temperature));
      temperatureNode.setProperty("sensor_ok").send("1");
      lastTemperatureSent = millis();
      retries = maxRetries + 1;
      
      // assume if temperature works sensor is good :-)
      humidity = getHumidity();
      Homie.getLogger() << "Humidity: " << humidity << " %" << endl;
      humidityNode.setProperty("percent").send(String(humidity));
      if ( deepSleepEnabled ) {
        Homie.prepareToSleep();
      }
    }
  }
}



/*
* Checks the battery voltage
*/
void checkBattery() {
  float batteryVoltage = ESP.getVcc() / 1024.00f; 
  Homie.getLogger() << "  Battery: " << batteryVoltage << "V" << endl;
  batteryNode.setProperty("voltage").send(String(batteryVoltage));
}

/**
 * Checks if the power / switch / door is open waits 5 seconds before a change occurs (again) 
 */
void checkDoorPowerState() {
  // last change needs to be 5 seconds away
  if ( lastPowerDoorCheckTime == 0 || millis() - lastPowerDoorCheckTime >= 3000UL ) {
  
    int sensorValue = digitalRead(DOOR_PIN);
    Homie.getLogger() << endl << "# Check Door and Power:" << endl;
    door_current_state = sensorValue;
    String doorPublishValue = door_current_state == HIGH ? "true" : "false";
    doorNode.setProperty("open").send(doorPublishValue);
    Homie.getLogger() << "DOOR open is: "  << doorPublishValue  << endl;
    
    int powerSensorValue = digitalRead(POWER_PIN);
    power_current_state = powerSensorValue;
    String powerPublishValue = power_current_state == HIGH ? "true" : "false";
    powerNode.setProperty("on").send(powerPublishValue);
    Homie.getLogger() << "POWER on is:  "  << powerPublishValue  << endl;
    Homie.getLogger() << endl << "# Check Door and Power:" << endl;

    lastPowerDoorCheckTime =  millis();
    
    
    checkBattery();

  }
  // else {
  //   Homie.getLogger() << "NO CHECK" << millis() << " " << lastPowerDoorCheckTime << endl;
  // }
  
}


void loopHandler() {
  checkDoorPowerState();
  checkDHT11();
}

void loop() {
  Homie.loop();
}
