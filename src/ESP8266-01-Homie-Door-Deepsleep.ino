#include <Homie.h>
#include <DHT.h>
#include <FS.h>
#include <ArduinoJson.h>

/*
#define DHTPIN 1 //  D1  (TX PIN) // using GIPO0 collides with flashing
#define DOOR_PIN 2 // D2
#define POWER_PIN 3 // D3 (RX PIN - be careful when flashing)
*/
#define DHTPIN 2 //  GPIO02
#define DOOR_PIN 0 // D1  (TX PIN) // using GIPO0 collides with flashing2
#define POWER_PIN 3 // D3 (RX PIN - be careful when flashing)


#define DHTTYPE DHT22   // Es handelt sich um den DHT22 Sensor
#define FW_NAME "esp-01-deepsleep-werkstatt"
#define FW_VERSION "2.0"


/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" FW_NAME "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" FW_VERSION "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */


// config file to store config for reset / reboot
const char* UH27_CONFIG_FILENAME = "uh27/config.json";
const unsigned int JSON_CONFIG_MAX_BUFFER_SIZE = 1024;
const char* CONFIG_VERSION = "1.0";

// comment
const int PIN_RELAY = LED_BUILTIN;
const int TEMPERATURE_INTERVAL = 5;

int temperature_interval = TEMPERATURE_INTERVAL;
unsigned long lastTemperatureSent = 0;
unsigned long lastRetry = 0;

int maxRetries = 5;
int retries = 0;


/* DOOR STUFF */
unsigned long lastPowerDoorCheckTime = 0;
int door_current_state = LOW;      // the current state of the output pin

/** POWER CONNECTED //  LIGHTS ON 
*/
unsigned long lastPowerDoor = 0;
int power_current_state = LOW;   

// battery
double battery_voltage;

/* USE RETAINED MESSAGE TO GUARANTEE PICKUP OF MESSAGE
iMac:~ $ mosquitto_pub -r -h 10.0.2.2 -t "homie/6001944981b0/deepsleep/sleeptime/set" -m "7"
iMac:~ $ mosquitto_pub -r -h 10.0.2.2 -t "homie/6001944981b0/deepsleep/enabled/set" -m "1"
iMac:~ $ mosquitto_pub -r -h 10.0.2.2 -t "homie/6001944981b0/deepsleep/enabled/set" -m "0"
iMac:~ $ mosquitto_pub  -h 10.0.2.2 -t "homie/6001944981b0/deepsleep/enabled/set" -m ""
*/
/* DEEPSLEEP */
const int DEFAULT_DEEP_SLEEP_TIME = 5;
int deepsleeptime = DEFAULT_DEEP_SLEEP_TIME;
int deepSleepEnabled = 1;


HomieNode lightNode("light", "switch");
HomieNode temperatureNode("temperature", "temperature");
HomieNode humidityNode("humidity", "humidity");
HomieNode doorNode("door", "door");
HomieNode powerNode("power", "power");
HomieNode batteryNode("battery", "battery");
HomieNode deepsleepNode("deepsleep", "deepsleep");

// Temperature
DHT dht(DHTPIN, DHTTYPE); 
// if you nee getVCC() turn this on
ADC_MODE(ADC_VCC);

void setupHandler() {

  doorNode.advertise("door");
  
  powerNode.advertise("power");

  lightNode.advertise("on").settable(lightOnHandler);
  lightNode.setProperty("on").send("false");
  
  temperatureNode.advertise("delay").settable(delayHandler);
  temperatureNode.setProperty("delay").send(String(temperature_interval));
 
  humidityNode.advertise("delay").settable(delayHandler);
  humidityNode.setProperty("delay").send(String(temperature_interval));

  deepsleepNode.advertise("sleeptime").settable(deepsleepHandler);
  deepsleepNode.setProperty("sleeptime").send(String(deepsleeptime));

  deepsleepNode.advertise("enabled").settable(deepsleepEnabledHandler);
  deepsleepNode.setProperty("enabled").send(String(deepSleepEnabled));
  
  temperatureNode.setProperty("unit").send("c");
  humidityNode.setProperty("unit").send("%");
  batteryNode.setProperty("unit").send("V");
  deepsleepNode.setProperty("unit").send("s");
}



bool delayHandler(const HomieRange& range, const String& value) { 
  
  Homie.getLogger() << "MQTT Input :: Got Delay " << value << endl;;
  int delay = value.toInt();
  temperature_interval = delay;
  temperatureNode.setProperty("delay").send(value);
  humidityNode.setProperty("delay").send(value);
  Homie.getLogger() << "   Delay is " << delay << endl;

  return true;
}





bool lightOnHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  lightNode.setProperty("on").send(value);
  Homie.getLogger() << "MQTT Input :: Light is " << (on ? "on" : "off") << endl;

  return true;
}


/*
*
*/
bool deepsleepEnabledHandler(const HomieRange& range, const String& value) { 
  
  Homie.getLogger() << "MQTT Input :: Got Deepsleep Enabled " << value << endl;;
  int enabled = value.toInt();
  deepSleepEnabled  = enabled;
  deepsleepNode.setProperty("enabled").send(String(enabled));

  Homie.getLogger() << "   Deepsleep Enabled value  " << deepSleepEnabled << endl;
  saveConfig();

  return true;
}

/*
*
*/
bool deepsleepHandler(const HomieRange& range, const String& value) { 
  
  Homie.getLogger() << "MQTT Input :: Got Deepsleep value (seconds) " << value << endl;;
  int delay = value.toInt();
  deepsleeptime  = delay;
  deepsleepNode.setProperty("sleeptime").send(value);

  Homie.getLogger() << "   deepsleep value  " << delay << endl;
  saveConfig();

  return true;
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


void setup() {
  // use RX as GPIO
//  Serial.begin(115200, SERIAL_8N1, SERIAL_);
  //Serial.setTimeout(2000);
  // Wait for serial to initialize.
  //while(!Serial) { }

  //Serial << endl << endl;
  
  initFS();
  readConfig();

  pinMode(DOOR_PIN, INPUT);
  pinMode(POWER_PIN, INPUT);
  
  //pinMode(PIN_RELAY, OUTPUT);
  //digitalWrite(PIN_RELAY, LOW);
 
  // uncomment for ESP-01
  Homie.disableLedFeedback(); // needed for deepsleep
//  Homie.setLedPin(10, HIGH);
  Homie.disableResetTrigger();
  Homie_setFirmware(FW_NAME, FW_VERSION);
  Homie_setBrand("mercatis-iot");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  
  Homie.onEvent(onHomieEvent);
  Homie.disableLogging();
  Homie.setup();
  dht.begin(); //DHT11 Sensor starten

}

float getTemperature() {  
  float temperature = dht.readTemperature();
  return temperature;
}

float getHumidity() {
  float humidity = dht.readHumidity(); 
 return humidity;
}

/**
 * Publish Data 
 */
void publishData() {

  float temperature = 0;
  float humidity = 0;
  lastTemperatureSent =  millis();

  // try getting temperature -- sometimes there is no answer from the sensor
  temperature = getTemperature();
  if ( isnan(temperature) || temperature == 0 ) {
    retries++;
    Homie.getLogger() << "  Temperature not available retrying " << retries << ". time " << " max Retries = " << lastRetry << "... " << millis() << endl;
    lastRetry = millis();
    temperatureNode.setProperty("sensor_ok").send("0");
  }
  else {

    Homie.getLogger() << "  Temperature: " << temperature << " °C" << endl;
  
    temperatureNode.setProperty("degrees").send(String(temperature));
    temperatureNode.setProperty("sensor_ok").send("1");
    lastTemperatureSent = millis();
    retries = maxRetries + 1;
  
    humidity = getHumidity();  // assume if temperature works sensor is good :-)
    Homie.getLogger() << "  Humidity: " << humidity << " %" << endl;
    humidityNode.setProperty("percent").send(String(humidity));
  


  }
}


void checkDHT11() {
  
  if (  ( retries < maxRetries && millis() - lastRetry >= 3000UL ) ||
        millis() - lastTemperatureSent >= temperature_interval * 1000UL ) {

    // successful the lasttime, new game
    if ( retries > maxRetries ) {
      retries = 0;
    } else if ( retries == maxRetries ) {
      Homie.getLogger() << "  # Maximum retries reached - Sensor not reachable - giving up .." << endl;
      temperatureNode.setProperty("sensor_ok").send("0");
      retries++;
      return;
    }
    Homie.getLogger() << endl << "* Regular Sensor readings" << endl;
    publishData();

  }
}

void checkDHT11DeepSleep() {
  // only once should be done in 5 seconds
  if (  millis() - lastTemperatureSent >= temperature_interval * 1000UL ) {

    Homie.getLogger() << endl << "* Deepsleep Sensor readings" << endl; 
    publishData();
    // uncomment ESP-01
    
    Homie.getLogger() << "  Prepare Deepsleep for " << deepsleeptime << " seconds " << endl;    
    Homie.prepareToSleep();
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
  if ( millis() - lastPowerDoorCheckTime >= temperature_interval * 1000UL ) {
  
    int sensorValue = digitalRead(DOOR_PIN);
    Homie.getLogger() << endl << "* Check Door and Power:" << endl;
    door_current_state = sensorValue;
    String doorPublishValue = door_current_state == HIGH ? "1" : "0";
    doorNode.setProperty("open").send(doorPublishValue);
    Homie.getLogger() << "  DOOR STATE="  << door_current_state << " ... " << endl;
    
    int powerSensorValue = digitalRead(POWER_PIN);
    power_current_state = powerSensorValue;
    String powerPublishValue = power_current_state == HIGH ? "1" : "0";
    powerNode.setProperty("on").send(powerPublishValue);
    Homie.getLogger() << "  power STATE="  << power_current_state << " ... " << endl;
    
    lastPowerDoorCheckTime =  millis();
    
    
    checkBattery();

  }
  //Homie.getLogger() << "*** IN PIN " << sensorValue << " ... " << endl;
  //delay(300);
  
}


//float getTemperature() {
 // sensors.requestTemperatures(); // Send the command to get temperature readings 
// return sensors.getTempCByIndex(0);
//}

void loopHandler() {
  //uncomment for deepsleep
  checkDoorPowerState();
  if ( deepSleepEnabled == 1) {
    checkDHT11DeepSleep();
  }
  else {
    checkDHT11();
  }
}

void loop() {
  Homie.loop();
}



/**************************************************************************/
/* FILESYSTEM */
/**************************************************************************/


/**
 * 
 */
void initFS() {
  bool result = SPIFFS.begin();
  if ( !result ) {
      //Serial.println("COULD NO Mount Filesystem");
  }
  else {
      //Serial.println("Mounted successfully.");
  }
}


/**
* 
*/
void saveConfig() {
  // open the file in write mode
  File f = SPIFFS.open(UH27_CONFIG_FILENAME, "w");
  if (!f) {
    //Serial.println("file creation failed");
  }
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["version"] = CONFIG_VERSION;
  root["sleeptime"] = deepsleeptime;
  root["deepSleepEnabled"] = deepSleepEnabled;
  String output;
  root.printTo(output);
  // now write two lines in key/value style with  end-of-line characters
  // char json[] =  "{\"sleeptime\":\"33\",\"version\":\"1.0\"}";
  f.println(output);
  f.close();
  //Serial.println("Updated UH27 Config.");
}

/**
* 
*/
bool readConfig() {
  File configFile = SPIFFS.open(UH27_CONFIG_FILENAME, "r");

  if (!configFile) {
    //Serial.println("✖ Cannot open config file");
    return false;
  }

  size_t configSize = configFile.size();
  
  std::unique_ptr<char[]> buf(new char[configSize]);
  configFile.readBytes(buf.get(), configSize);
  
  StaticJsonBuffer<JSON_CONFIG_MAX_BUFFER_SIZE> jsonBuffer;
  JsonObject& parsed_json = jsonBuffer.parseObject(buf.get());
  if (!parsed_json.success()) {
    //Serial.println("✖ Invalid or too big config file");
    return false;
  }

  int delay = parsed_json["sleeptime"]; 
  deepsleeptime = delay;
  // Serial.print("sleeptime : "); 
  // Serial.println(delay);


  int deepSleepOn = parsed_json["deepSleepEnabled"]; 
  deepSleepEnabled = deepSleepOn;
  // Serial.print("Deepsleep On : "); 
  // Serial.println(deepSleepEnabled);

  const char* version = parsed_json["version"]; 
  // Serial.print("Version : ");
  // Serial.println(version);
}


/*
void getInitialConfig() {

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root.add("wifi")
  root["sensor"] = "gps";
  root["time"] = 1351824120;
  "name": "The kitchen light",
"device_id": "kitchen-light",
"wifi": {
  "ssid": "Network_1",
  "password": "I'm a Wi-Fi password!",
  "bssid": "DE:AD:BE:EF:BA:BE",
  "channel": 1,
  "ip": "192.168.1.5",
  "mask": "255.255.255.0",
  "gw": "192.168.1.1",
  "dns1": "8.8.8.8",
  "dns2": "8.8.4.4"
},
"mqtt": {
  "host": "192.168.1.10",
  "port": 1883,
  "base_topic": "devices/",
  "auth": true,
  "username": "user",
  "password": "pass"
},
"ota": {
  "enabled": true
},
"settings": {
  "percentage": 55
}
}
*/