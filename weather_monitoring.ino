#include <LiquidCrystal_I2C.h>

#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Pin Definitions
#define DHT_PIN D5          // DHT11 connected to NodeMCU pin D5 (GPIO14)
#define DHT_TYPE DHT11      // Specify the sensor type (DHT11 or DHT22)
#define MQ135_PIN D0        // MQ-135 sensor connected to NodeMCU analog pin A0
#define RAIN_SENSOR_PIN D1  // Rain sensor connected to NodeMCU pin D1 (GPIO5)
#define LED_PIN D2          // LED connected to NodeMCU pin D2 (GPIO4)

// GPS Pins
#define GPS_RX D6           // GPS RX connected to NodeMCU pin D6
#define GPS_TX D7           // GPS TX connected to NodeMCU pin D7

// Initialize sensors and peripherals
DHT dht(DHT_PIN, DHT_TYPE);                         // DHT sensor object
LiquidCrystal_I2C lcd(0x27, 16, 2);                 // LCD object
TinyGPSPlus gps;                                    // GPS object
SoftwareSerial gpsSerial(GPS_TX, GPS_RX);           // SoftwareSerial for GPS

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  gpsSerial.begin(9600);

  // Initialize DHT sensor
  dht.begin();

  // Initialize pins for Rain Sensor and LED
  pinMode(RAIN_SENSOR_PIN, INPUT); // Rain sensor pin as input
  pinMode(LED_PIN, OUTPUT);        // LED pin as output

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.print("System Starting");
  delay(2000);
  lcd.clear();

  Serial.println("Multi-Sensor System Initialized");
}

void loop() {
  // 1. Read DHT11 Sensor (Temperature and Humidity)
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  String dhtMessage;
  if (isnan(humidity) || isnan(temperature)) {
    dhtMessage = "DHT Error!";
    Serial.println("Failed to read from DHT sensor!");
  } else {
    dhtMessage = "T:" + String(temperature, 1) + "C H:" + String(humidity, 0) + "%";
    Serial.println(dhtMessage);
  }

  // 2. Read MQ-135 Sensor (Air Quality)
  int mq135Value = analogRead(MQ135_PIN);
  float mq135Voltage = mq135Value * (3.3 / 1023.0);
  String airQuality = (mq135Value < 100) ? "Clean" : (mq135Value < 400) ? "Moderate" : "Poor";

  Serial.print("MQ-135 | Value: "); 
  Serial.print(mq135Value);
  Serial.print(" | Voltage: ");
  Serial.print(mq135Voltage, 2); // Two decimal places
  Serial.print(" V | Quality: ");
  Serial.println(airQuality);

  // 3. Read Rain Sensor
  int rainStatus = digitalRead(RAIN_SENSOR_PIN);
  String rainMessage = (rainStatus == LOW) ? "Rain detected!" : "No rain.";
  if (rainStatus == LOW) {
    digitalWrite(LED_PIN, HIGH); // Turn on LED
  } else {
    digitalWrite(LED_PIN, LOW);  // Turn off LED
  }
  Serial.println(rainMessage);

  // 4. Read GPS Data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  String gpsMessage = "GPS Invalid!";
  if (gps.location.isValid()) {
    gpsMessage = "Lat:" + String(gps.location.lat(), 6) +
                 " Lon:" + String(gps.location.lng(), 6);
    Serial.println(gpsMessage);
  } else {
    Serial.println("Waiting for valid GPS data...");
  }

  // Display readings on LCD (rotate display content)
  lcd.clear();
  lcd.setCursor(0, 0); // First line
  lcd.print(dhtMessage);
  lcd.setCursor(0, 1); // Second line
  lcd.print(airQuality + " " + (rainStatus == LOW ? "Rain" : "No Rain"));
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0); // First line
  lcd.print("GPS:");
  lcd.setCursor(0, 1); // Second line
  lcd.print(gpsMessage);
  delay(2000);
}
