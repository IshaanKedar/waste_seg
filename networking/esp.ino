#include <ESP32Servo.h>
#include <DHT.h>
#include <NewPing.h>


// Define Pins
#define TRIG_SOLID 5
#define ECHO_SOLID 18
#define TRIG_LIQUID 2
#define ECHO_LIQUID 4
#define DHT_PIN 21
#define DHT_TYPE DHT11
#define SERVO_PIN 23  // Servo Motor Pin
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701


// Initialize Sensors & Servo
DHT dht(DHT_PIN, DHT_TYPE);
// NewPing solidWasteSensor(TRIG_SOLID, ECHO_SOLID, 200);
// NewPing liquidWasteSensor(TRIG_LIQUID, ECHO_LIQUID, 200);
Servo wasteServo;

float solidLevel,liquidLevel;
long duration_SOLID,duration_LIQUID;

void setup() {
  Serial.begin(115200);
  dht.begin();
  wasteServo.attach(SERVO_PIN);
  wasteServo.write(90);  // Neutral position

  pinMode(TRIG_SOLID, OUTPUT);
  pinMode(ECHO_SOLID, INPUT);
  pinMode(TRIG_LIQUID, OUTPUT);
  pinMode(ECHO_LIQUID, INPUT);
}


void loop() {
  // Trigger Solid Waste Sensor
  digitalWrite(TRIG_SOLID, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_SOLID, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_SOLID, LOW);
  duration_SOLID = pulseIn(ECHO_SOLID, HIGH);
  solidLevel = duration_SOLID * SOUND_SPEED / 2;

  delay(50);  // Small delay between measurements to avoid signal interference

  // Trigger Liquid Waste Sensor
  digitalWrite(TRIG_LIQUID, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_LIQUID, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_LIQUID, LOW);
  duration_LIQUID = pulseIn(ECHO_LIQUID, HIGH);
  liquidLevel = duration_LIQUID * SOUND_SPEED / 2;

  // Read DHT11 data
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Display sensor readings
  Serial.print("Solid Waste Level: "); Serial.print(solidLevel); Serial.println(" cm");
  Serial.print("Liquid Waste Level: "); Serial.print(liquidLevel); Serial.println(" cm");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");

  // Segregation Logic
  if (temperature > 30.0 && humidity > 60.0) {
    Serial.println("Organic Waste Detected");
    wasteServo.write(45);  // Move flap to Organic Waste section
  } else {
    Serial.println("Inorganic Waste Detected");
    wasteServo.write(135); // Move flap to Inorganic Waste section
  }

  delay(3000);  // Allow time for waste to settle
  wasteServo.write(90);  // Reset to neutral position

  Serial.println("--------------------");
  delay(2000);
}