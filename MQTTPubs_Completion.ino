#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include "HX711.h"

// Replace with your network credentials
const char* ssid = "Souls";
const char* password = "T6mw-YSej-syim-om4i";
// Replace with your MQTT Broker's IP address or hostname
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);

// Ultrasonic sensor 1 pins
const int trigPin1 = 13;
const int echoPin1 = 12;

// Ultrasonic sensor 2 pins
const int trigPin2 = 21;
const int echoPin2 = 19;

// DHT11 sensor pin and type
#define DHTPIN 22
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

// Define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// Water level sensor pins (ADC1 pins)
const int waterSensorPin1 = 34;
const int waterSensorPin2 = 33;

// Photoresistor sensor pin
const int photoresistorPin = 32; // Use an appropriate analog pin

// Weight sensor pins
const int LOADCELL_DOUT_PIN = 15;
const int LOADCELL_SCK_PIN = 2;

HX711 scale;

// Use a predetermined calibration factor
float calibration_factor = 2280.0;  // Replace with your own calibration factor
float last_known_weight = 0;

long duration1;
float distanceCm1;
float distanceInch1;

long duration2;
float distanceCm2;
float distanceInch2;

float humidity;
float temperature;
int waterLevel1;
int waterLevel2;

unsigned long lastDHTReadTime = 0;
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 1000; // Publish interval in milliseconds

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Subscribe to topics if needed
      // client.subscribe("yourTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200); // Starts the serial communication

  // Initialize ultrasonic sensor 1
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input

  // Initialize ultrasonic sensor 2
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input

  pinMode(waterSensorPin1, INPUT); // Sets the waterSensorPin1 as an Input
  pinMode(waterSensorPin2, INPUT); // Sets the waterSensorPin2 as an Input

  pinMode(photoresistorPin, INPUT); // Sets the photoresistorPin as an Input

  dht.begin(); // Initialize DHT sensor

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Initialize weight sensor
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);  // Set the predetermined calibration factor
  scale.tare();  // Reset scale to 0

  // Wait for a moment to ensure everything is initialized properly
  delay(1000);
}

void readUltrasonicSensor1() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);

  duration1 = pulseIn(echoPin1, HIGH);
  distanceCm1 = 250 - (duration1 * SOUND_SPEED / 2);
  distanceInch1 = distanceCm1 * CM_TO_INCH;

  Serial.print("Ultrasonic Sensor 1 Distance (cm): ");
  Serial.println(distanceCm1);
  Serial.print("Ultrasonic Sensor 1 Distance (inch): ");
  Serial.println(distanceInch1);

  // Convert distance to string
  String ultrasonicData1 = String(distanceCm1);

  // Publish to both topics
  client.publish("topic/ultrasonic1", ultrasonicData1.c_str());
  client.publish("topic/test", ultrasonicData1.c_str()); // Publish the same data to topic/test
}

void readUltrasonicSensor2() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  duration2 = pulseIn(echoPin2, HIGH);
  distanceCm2 = 250 - (duration2 * SOUND_SPEED / 2);
  distanceInch2 = distanceCm2 * CM_TO_INCH;

  Serial.print("Ultrasonic Sensor 2 Distance (cm): ");
  Serial.println(distanceCm2);
  Serial.print("Ultrasonic Sensor 2 Distance (inch): ");
  Serial.println(distanceInch2);

  // Publish ultrasonic sensor 2 distance to MQTT
  String ultrasonicData2 = String(distanceCm2);
  client.publish("topic/ultrasonic2", ultrasonicData2.c_str());
}

void readDHTSensor() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");

    // Publish humidity and temperature to MQTT
    String humidityData = String(humidity);
    String temperatureData = String(temperature);

    client.publish("topic/humidity", humidityData.c_str());
    client.publish("topic/temperature", temperatureData.c_str());
  }
}

void readWaterLevelSensors() {
  // Read the raw values from the water level sensors
  waterLevel1 = analogRead(waterSensorPin1);
  waterLevel2 = analogRead(waterSensorPin2);

  // Constrain the values to be between 0 and 1000
  waterLevel1 = constrain(waterLevel1, 0, 1000);
  waterLevel2 = constrain(waterLevel2, 0, 1000);

  Serial.print("Water Level Sensor 1 Value: ");
  Serial.println(waterLevel1);
  Serial.print("Water Level Sensor 2 Value: ");
  Serial.println(waterLevel2);

  // Publish water level data to MQTT
  String waterLevelData1 = String(waterLevel1);
  String waterLevelData2 = String(waterLevel2);

  client.publish("topic/waterlevel1", waterLevelData1.c_str());
  client.publish("topic/waterlevel2", waterLevelData2.c_str());
}

void readPhotoresistorSensor() {
  int sensorValue = analogRead(photoresistorPin);

  // Invert the sensor value (assuming 10-bit ADC)
  sensorValue = 1023 - sensorValue;

  // Ensure the sensor value is not negative
  if (sensorValue < 0) {
    sensorValue = 0;
  }

  Serial.print("Photoresistor value: ");
  Serial.println(sensorValue);

  // Publish light sensor value to MQTT
  String lightSensorData = String(sensorValue);
  client.publish("topic/lightsensor", lightSensorData.c_str());
}

void readWeightSensor() {
  float weight = scale.get_units(10);  // Average of 10 readings

  // Check for weight change
  if (abs(weight - last_known_weight) > 0.01) {  // If weight changes significantly
    Serial.print("Weight: ");
    Serial.print(weight);
    Serial.println(" kg");

    // Publish weight to MQTT
    String weightData = String(weight);
    client.publish("topic/weight", weightData.c_str());

    // If the weight is effectively zero, recalibrate
    if (weight < 0.01) {
      scale.tare();  // Reset scale to 0
      Serial.println("Scale recalibrated to zero.");
    }

    last_known_weight = weight;
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read and publish sensor data
  readUltrasonicSensor1();
  delay(100); // Short delay between publishes

  readUltrasonicSensor2();
  delay(100); // Short delay between publishes

  unsigned long currentTime = millis();
  if (currentTime - lastDHTReadTime >= 2000) {
    readDHTSensor();
    lastDHTReadTime = currentTime;
  }

  readWaterLevelSensors();
  delay(100); // Short delay between publishes

  readPhotoresistorSensor();
  delay(100); // Short delay between publishes

  readWeightSensor();
  delay(100); // Short delay between publishes

  delay(100); // Short delay to avoid overwhelming the broker
}
