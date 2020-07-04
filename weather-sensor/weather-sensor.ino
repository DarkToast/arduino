#include "ESP8266WiFi.h"
#include "DHT.h"
#include "PubSubClient.h"
#include <SFE_BMP180.h>
#include <string>
#include <math.h>

/************************* WiFi Access Point *******************************/
#define WLAN_SSID       "Banane"
#define WLAN_PASS       "4Ftsid1n3rW"
WiFiClient espClient;

/************************* MQTT Connection *********************************/
#define MQTT_HOST "192.168.0.11"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "d-1"
#define MQTT_TOPIC "/d/1/th"
PubSubClient client;

/************************* DHT Sensors *************************************/
#define DHTPIN D4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

/**************************/
#define ALTITUDE 62.0
SFE_BMP180 bmp;

/************************* APP Defines *************************************/
struct DhtMeasurement {
  float temperature;
  float humidity;
};

struct BmpMeasurement {
  float temperature;
  float absolutePressure;
  float relativePressure;
  float altitude;
};

struct Measurement {
  DhtMeasurement dhtM;
  BmpMeasurement bmpM;
};


/************
   Connects to WiFi
   Returns the own IP address
 ************/
IPAddress connectToWifi() {
  Serial.print("Connecting to WiFi");
  WiFi.persistent(false);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected, IP address: ");
  IPAddress address = WiFi.localIP();
  Serial.println(address);
  return address;
}

/************
   Connects to the MQTT broker
 ************/
void checkMqttConnection() {

  if (!client.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(MQTT_HOST);
    Serial.print(":");
    Serial.print(MQTT_PORT);
  }

  // Attempt to connect
  while (!client.connected()) {
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println(" connected");
    } else {
      Serial.print(".");
      delay(5000);
    }
  }
}

/************
  Connects to the temperature sensor
  and read temperature and humidity.
************/
DhtMeasurement readDhtSensor() {
  struct DhtMeasurement measurement;
  int tries = 1;

  Serial.print("Measure temperature and humidity ");
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  while (isnan(humidity) || isnan(temperature)) {
    if (tries >= 3) {
      Serial.println(" - Failed to read temperature.");
      measurement.humidity = -1;
      measurement.temperature = -1;
      return measurement;
    }

    delay(2000);
    Serial.print(".");
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    tries++;
  }

  Serial.print(temperature);
  Serial.print(" °C ");
  Serial.print(humidity);
  Serial.print(" %");
  Serial.println();

  measurement.humidity = humidity * 100;
  measurement.temperature = temperature * 100;
  return measurement;
}

/************
  Connects to the air pressure sensor
  and read temperature and air pressure.
************/
BmpMeasurement readBmpSensor() {
  int tries = 1;
  struct BmpMeasurement measurement;
  measurement.temperature = -1;
  measurement.absolutePressure = -1;
  measurement.relativePressure = -1;
  measurement.altitude = -1;

  double T, P, P0;

  char* errorMessage = "error while measure the bmp sensor\n";
  char status;

  Serial.println("Measure temperature and air pressure ");

  status = bmp.startTemperature();
  if (status == 0) {
    Serial.println(errorMessage);
    return measurement;
  }

  delay(status);

  status = bmp.getTemperature(T);
  if (status == 0) {
    Serial.println(errorMessage);
    return measurement;
  }

  Serial.print("temperature: ");
  Serial.print(T, 2);
  Serial.println(" deg C, ");

  status = bmp.startPressure(3);
  if (status == 0) {
    Serial.println(errorMessage);
    return measurement;
  }

  delay(status);

  status = bmp.getPressure(P, T);
  if (status == 0) {
    Serial.println(errorMessage);
    return measurement;
  }

  Serial.print("absolute pressure: ");
  Serial.print(P, 2);
  Serial.println(" mb, ");

  P0 = bmp.sealevel(P, ALTITUDE);
  Serial.print("relative (sea-level) pressure: ");
  Serial.print(P0, 2);
  Serial.println(" mb, ");

  measurement.temperature = T * 100;
  measurement.absolutePressure = round(P * 10);
  measurement.relativePressure = round(P0 * 10);
  measurement.altitude = ALTITUDE;

  return measurement;
}

/************
   Publish message to MQTT topic
 ************/
void publish(Measurement m) {
  char log[40];
  Serial.println("Publish to topic.");

  short message[5];
  message[0] = (m.dhtM.temperature + m.bmpM.temperature) / 2;
  message[1] = m.dhtM.humidity;
  message[2] = m.bmpM.altitude;
  message[3] = m.bmpM.absolutePressure;
  message[4] = m.bmpM.relativePressure;
  client.publish(MQTT_TOPIC, (uint8_t*) &message, 10);
}


/*#####################
   MAIN METHODS - setup and loop
  ####################*/
void setup() {
  Serial.begin(9600);
  Serial.println("Device started");

  client.setClient(espClient);
  client.setServer("192.168.0.11", 1883);

  connectToWifi();
  delay(500);


  while (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    delay(5000);
  }
}

unsigned long lastMsg = 0;

void loop() {


  checkMqttConnection();

  delay(300);

  DhtMeasurement dhtM = readDhtSensor();
  if (dhtM.temperature == -1) {
    Serial.println("DHT measurement failed. Ignoring!");
    return;
  }

  BmpMeasurement bmpM = readBmpSensor();
  if (bmpM.temperature == -1) {
    Serial.println("BMP measurement failed. Ignoring!");
    return;
  }

  Measurement m = Measurement();
  m.dhtM = dhtM;
  m.bmpM = bmpM;

  publish(m);
  client.loop();

  Serial.println("Going into deep sleep for 5 minutes");
  delay(60000 * 5);
  // ESP.deepSleep(60e6 * 1); // 60e6 is 60 seconds --> 5 minutes
}
