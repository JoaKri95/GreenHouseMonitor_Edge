#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_DotStar.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_LTR329_LTR303.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <MQTT.h>
#include <Pushsafer.h>
#include <WiFiClientSecure.h>

/*
const char ssid[] = "ssid";
const char pass[] = "pass";
const char mqtt_username[] = "YourUserName";
const char mqtt_password[] = "YourPassword";
const char mqtt_server[]   = "mqtt_server_address";
*/
const char ssid[] = "";
const char pass[] = "";


// MQTT Broker
const char* mqtt_server = "";
const int mqtt_port = 8883; // Port for SSL/TLS
const char* mqtt_username = ""; // If applicable
const char* mqtt_password = ""; // If applicable


#define HALL_SENSOR_PIN 1
#define NUMPIXELS 1
#define DATAPIN    33
#define CLOCKPIN   21
#define PushsaferKey ""   // Private Key from: http://pushsafer.com //bytt ut med egen nøkkel og egen device id lenger ned i sendDeviantValueAlertToPhone funksjonen

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_LTR329 ltr = Adafruit_LTR329();
WiFiClient networkClient;
WiFiMulti wiFiMulti;


Pushsafer pushsafer(PushsaferKey, networkClient);

WiFiClientSecure net;
MQTTClient mqttClient;

const char* root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc= 
-----END CERTIFICATE-----
)EOF";

//Global variables 

String greenHouseLocationOneID = "1"; 
String greenHouseLocationOne = "Oslo";

int light;

unsigned long lastMillis = 0;

unsigned long lastTime = 0;


const float maxHumidityValue = 40;
const float minHumidityValue  = 20; 

const float maxTemperatureValue = 31;
const float minTemperatureValue = 16; 

const float maxLuxValue = 100000;
const float minLuxValue = 32000;


unsigned long startTimeHumidityExceedsTreshold = 0;
unsigned long startTimeTemperatureExceedsTreshold = 0;
unsigned long startTimeLuxValueExceedsTreshold = 0;


bool humidityTresholdExceeded = false;
bool temperatureTresholdExceeded = false;
bool luxValueTresholdExceeded = false;

bool temperatureAlertSent = false;
bool humidityAlertSent = false;
bool lightAlertSent = false;


//MagnetSensor
bool doorAlertSent = false;
bool magnetRemovedStatus = false; 
unsigned long magnetRemovalStartTime = 0;




void stayConnectedToWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Wifi connection lost");
    strip.setPixelColor(0, 255, 0, 0); // red LED to show connection problem
    strip.show(); // Update LED with new contents

    // if connection is lost, wait 3 seconds and try connecting again
    while (WiFi.status() != WL_CONNECTED) {
      delay(3000);
      Serial.print(".");
      WiFi.reconnect();
    }
    Serial.print("Wifi connection regained!");
  }
}

/*
void stayConnectedToMqtt() {
  if( !mqttClient.connected() )
  {
    mqttClient.begin(mqtt_server, networkClient);

    String clientId = "ESP32Client-"; 
      clientId += String(random(0xffff), HEX);

    Serial.print("\nConnecting to MQTT...");
    while (!mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("\nMQTT Connected!");
  } else {
    mqttClient.loop();
  }
}
*/

void stayConnectedToMqtt() {
  net.setCACert(root_ca); // Set CA certificate
  mqttClient.begin(mqtt_server, mqtt_port, net);

  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");

    if (mqttClient.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");
      // Subscribe or perform additional setup
    } else {
      Serial.print("Failed with state ");
      Serial.print(mqttClient.lastError());
      Serial.print(", ");
      Serial.print(mqttClient.returnCode());
      delay(2000);
    }
  }
}

unsigned int getLuxValue(uint16_t CH0, uint16_t CH1){
  double CH0_double = static_cast<double>(CH0);
  double CH1_double = static_cast<double>(CH1);

    unsigned int lux = 0;
    byte integrationTime = 0x01;
		byte gain = 0x00;
		double ratio, d0, d1;
		uint ALS_GAIN[8] = {1, 2, 4, 8, 1, 1, 48, 96};
		float ALS_INT[8] = {1.0, 0.5, 2.0, 4.0, 1.5, 2.5, 3.0, 3.5};
		// Determine if either sensor saturated (0xFFFF)
		// If so, abandon ship (calculation will not be accurate)
		if ((CH0 == 0xFFFF) || (CH1 == 0xFFFF))
		{
			lux = 0.0;
			return lux;
		}

		// Convert from unsigned integer to floating point
		d0 = CH0;
		d1 = CH1;

		// We will need the ratio for subsequent calculations
		ratio = d1 / (d0 + d1);

		// Determine lux per datasheet equations:
		if (ratio < 0.45)
		{
			lux = ((1.7743 * d0) + (1.1059 * d1)) / ALS_GAIN[gain] / ALS_INT[integrationTime];
		}

		else if (ratio < 0.64)
		{
			lux = ((4.2785 * d0) - (1.9548 * d1)) / ALS_GAIN[gain] / ALS_INT[integrationTime];
		}

		else if (ratio < 0.85)
		{
			lux = ((0.5926 * d0) + (0.1185 * d1)) / ALS_GAIN[gain] / ALS_INT[integrationTime];
		}

		else
			lux = 0;

    light = lux;
	  return lux;
}


void sendDeviantValueAlertToPhone(bool alertType, String title, String message){
  if (!alertType){
          struct PushSaferInput input;
          input.message = message;
          input.title = title;
          input.sound = "6";
          input.vibration = "1";
          input.icon = "1";
          input.iconcolor = "#00FF00";
          input.priority = "1";
          input.device = "";      // Device ID:  http://pushsafer.com  //Bytt ut med egen ID
          pushsafer.sendEvent(input);
  }
}

// void doorAlertAndWeatherStatus(){  
//     if ((wiFiMulti.run() == WL_CONNECTED)) { 
 
//     HTTPClient http;
 
//     //Min ID er brukt her, og kan brukes for å teste løsningen.
//     http.begin("https://api.openweathermap.org/data/2.5/weather?q="+String(greenHouseLocationOne)+"&APPID=806d2e271d26eedf6c8577e68d5ebaa1"); //Specify the URL
//     int httpCode = http.GET();  //Make the request
 
//     if (httpCode > 0) { //Check for the returning code
//         if(httpCode == HTTP_CODE_OK){
//         String payload = http.getString();

//         const size_t capacity = payload.length() * 2;
//         DynamicJsonDocument doc(capacity);
//         DeserializationError error = deserializeJson(doc, payload); 

//         if(error){

//           Serial.println(F("deserializeJson() failed with code "));
//           Serial.print(error.f_str());

//         } else {

//           String weatherMain = doc["weather"] [0] ["main"];
//           String weatherDetail = doc["weather"] [0] ["description"];
//           float mainTemperature = doc["main"]["temp"];
//           int mainHumidity = doc["main"]["humidity"];

//           sendDeviantValueAlertToPhone(doorAlertSent, "Door Alert!", "The door has been open for 5 minutes. Current temperature outside is: " + String(mainTemperature - 273.15) + "°C" + ". Humidity: " + String(mainHumidity) + "%");

//         }
//       }
//     } else {
//       Serial.printf("Error on HTTP GET: %s\n", http.errorToString(httpCode).c_str());
//     }
//     http.end(); //Free the resources
//   }
// }





void setup() {
  Serial.begin(115200);

  // To use the RGB LED, we need to "start it" using these methodss
  strip.begin();            // Initialize LED pins for output
  strip.setBrightness(20);  // Avoid blinding yourself
  strip.setPixelColor(0, 255, 255, 0); // Yellow LED to show connecting
  strip.show(); // Update LED with new contents

  pinMode( LED_BUILTIN, OUTPUT );
  pinMode( HALL_SENSOR_PIN, INPUT );

  // start up the sensor
  if (! sht31.begin(0x44)) {
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  // Connect to wifi
  Serial.print("\nConnecting to Wifi");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nWifi Connected!");

  delay(500);
  wiFiMulti.addAP(ssid, pass);

  if ( ! ltr.begin() ) {
    Serial.println("Couldn't find LTR sensor!");
    while (1) delay(10);
  }

  // Setup LTR sensor (see advanced demo in library for all options!)
  Serial.println("Found LTR sensor!");
  ltr.setGain(LTR3XX_GAIN_4);
  ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
  ltr.setMeasurementRate(LTR3XX_MEASRATE_50);

}




void loop() {
  stayConnectedToWifi();
  stayConnectedToMqtt();


      strip.setPixelColor(0, 0, 255, 0);
      strip.show();

    
  //Sensor readings
  uint16_t visible_plus_ir, infrared;
    if (ltr.newDataAvailable()) {
      bool valid = ltr.readBothChannels(visible_plus_ir, infrared);
      if (valid) {
        Serial.print("\tLux Value: ");
        Serial.print(getLuxValue(visible_plus_ir, infrared));
      }
    }

    bool sensorReadingDidWork = true;
    float temperature = sht31.readTemperature();
    Serial.print("Temp = "); Serial.print(temperature); Serial.print("\t\t");
    if (isnan(temperature)) {  // check if 'is not a number'
      sensorReadingDidWork = false;
    }
    
    float humidity = sht31.readHumidity();
    Serial.print("Hum = "); Serial.println(humidity);
    if (isnan(humidity)) {  // check if 'is not a number'
      sensorReadingDidWork = false;
    }

    if(isnan(light)){
      sensorReadingDidWork = false;
    }


  //Checking min max HUMIDITY levels
  if (humidity > maxHumidityValue || humidity < minHumidityValue) {
    if (!humidityTresholdExceeded) {
      startTimeHumidityExceedsTreshold = millis();
      Serial.println(startTimeHumidityExceedsTreshold);
      humidityTresholdExceeded  = true;
      Serial.println("Humidity has gone over or under the treshold! StartTime saved");
    }
    if (millis() - startTimeHumidityExceedsTreshold >  120000 ) {
      Serial.println("Humidity has gone over or under the treshold value for 2 minutes straight! Check status on website");
      
      sendDeviantValueAlertToPhone(humidityAlertSent, "Humidity Alert", "Humidity has gone over or under the treshold value for 2 minutes straight! Check status on website");
      humidityAlertSent = true;

      humidityTresholdExceeded  = false;
    }
  }
  else {
    humidityTresholdExceeded  = false;
    humidityAlertSent = true;
  }


//Checking min max Temperature levels
  if (temperature > maxTemperatureValue || temperature < minTemperatureValue) {
    if (!temperatureTresholdExceeded) {
      startTimeTemperatureExceedsTreshold = millis();
      Serial.println(startTimeTemperatureExceedsTreshold);
      temperatureTresholdExceeded  = true;
      Serial.println("Temperature has gone over or under the treshold!");
    }

    if (millis() - startTimeTemperatureExceedsTreshold >= 120000) {
      Serial.println("Temperature has gone over or under the treshold for 2 minutes straight, Check status on website");

      sendDeviantValueAlertToPhone(temperatureAlertSent, "Temperature Alert", "Temperature has gone over or under the treshold for 2 minutes straight, Check status on website");
      temperatureAlertSent = true;

      temperatureTresholdExceeded  = false;
    }
  }
  else {
    temperatureTresholdExceeded  = false;
    temperatureAlertSent = false;
  }

//Checking min max Light levels
  if (light > maxLuxValue || light < minLuxValue) {
    if (!luxValueTresholdExceeded) {
      startTimeLuxValueExceedsTreshold = millis();
      Serial.println(startTimeLuxValueExceedsTreshold);
      luxValueTresholdExceeded  = true;
      Serial.println("Lightlevel has gone over or under the treshold!");
    }
    if (millis() - startTimeLuxValueExceedsTreshold > 120000) {
      Serial.println("Lightlevel has gone over or under the treshold for 2 minutes straight! Check status on website");

      sendDeviantValueAlertToPhone(lightAlertSent, "Light Alert", "Lightlevel has gone over or under the treshold for 2 minutes straight! Check status on website");
      lightAlertSent = true;

      luxValueTresholdExceeded  = false;
    }
  }
  else {
    luxValueTresholdExceeded  = false;
    lightAlertSent = false;
  }


  bool isMagnetRemoved = digitalRead(HALL_SENSOR_PIN);
  digitalWrite(LED_BUILTIN, isMagnetRemoved);
  Serial.println(isMagnetRemoved);

  //Hallsensor to further protect our precious fruits
  if(isMagnetRemoved == 1){
    if(!magnetRemovedStatus){
      magnetRemovalStartTime = millis();
      Serial.println(magnetRemovalStartTime);
      magnetRemovedStatus  = true;
      Serial.println("The door is open! Saving startTime");
    }
    if(millis() - magnetRemovalStartTime > 120000 ){
      Serial.println("The door has been open for 2 minutes");
      //Funksjon for å sende varsel her:
      //doorAlertAndWeatherStatus();

      doorAlertSent = true;
      magnetRemovedStatus = false;
    }
  }
  else{
    magnetRemovedStatus = false;
    doorAlertSent = false;
  }
  
  

  if (millis() - lastMillis > 5000) {
    lastMillis = millis();

    if( sensorReadingDidWork ) {

      String mqttTopicWeSendDataTo = "GreenHouseMonitor/GreenHouseOne";

      String jsonData = "{\"greenHouseId\":"+String(greenHouseLocationOneID)+",\"temperature\":"+String(temperature)+",\"humidity\":"+String(humidity)+",\"light\":"+String(light)+"}";

      mqttClient.publish(mqttTopicWeSendDataTo, jsonData);
      Serial.print("Sent to MQTT: ");
      Serial.println(jsonData);
    }
  }


  delay(1000);
}