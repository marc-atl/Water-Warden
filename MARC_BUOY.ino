#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


#define SEALEVELPRESSURE_HPA 1013.25  // Standard sea level pressure in hPa

// BME280 sensor pins
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

// GPS pins
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
int UTC_OFFSET = -5;  // Offset for EST (UTC - 5)

// MPU6050 address
const int MPU_addr1 = 0x68;

// Global variables for BME280 and MPU6050
Adafruit_BME280 bme;         // BME280 sensor
TinyGPSPlus gps;             // GPS object
SoftwareSerial ss(RXPin, TXPin);
float calibration_value = 26.00;
const int numSamples = 200;

// Variables for MPU6050 data
float xa, ya, za, roll, pitch, yaw;
float gx, gy, gz;
int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
unsigned long previousTime;
float yawAngle = 0.0;

void setup() {
    Serial.begin(9600);
    ss.begin(GPSBaud);

    // Initialize BME280
   // if (!bme.begin(0x76)) {
     //   Serial.println("Could not find a valid BME280 sensor, check wiring!");
       // while (1);
    //}

    // Initialize MPU6050
    Wire.begin();
    Wire.beginTransmission(MPU_addr1);
    Wire.write(0x6B);  // Access power management register
    Wire.write(0);     // Wake up MPU6050
    Wire.endTransmission(true);

    previousTime = millis();  // Initialize previous time for yaw calculation

    Serial.println("Sensor setup complete!");
}

void loop() {
    if (ss.available() > 0) {
        gps.encode(ss.read());
        if (gps.location.isUpdated()) {
            displayEnvironmentalData();
            displayGPSData();
            displayMPU6050Data();
            delay(1000);  // Delay for data stabilization
        }
    }
}

void displayEnvironmentalData() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.print("Turbidity = ");
    Serial.print(analogRead(A1));
    Serial.println("");

    Serial.print("TDS = ");
    Serial.print(analogRead(A0));
    Serial.println("");

    float ph_value = calculatePH();
    Serial.print("pH = ");
    Serial.println(ph_value);
}

float calculatePH() {
    int buffer_arr[numSamples];
    for (int i = 0; i < numSamples; i++) {
        buffer_arr[i] = analogRead(A2);
        delay(15);
    }

    for (int i = 0; i < numSamples - 1; i++) {
        for (int j = i + 1; j < numSamples; j++) {
            if (buffer_arr[i] > buffer_arr[j]) {
                int temp = buffer_arr[i];
                buffer_arr[i] = buffer_arr[j];
                buffer_arr[j] = temp;
            }
        }
    }

    int medianValue = buffer_arr[numSamples / 2];
    float volt = (float)medianValue * 5.0 / 1024.0;
    float temperature = bme.readTemperature();
    float temp_correction_factor = 0.03 * (temperature - 25.0);
    return -5.70 * volt + calibration_value + temp_correction_factor;
}

void displayGPSData() {
    Serial.print("Latitude= ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Longitude= ");
    Serial.println(gps.location.lng(), 6);

    int hourEST = gps.time.hour() + UTC_OFFSET;
    if (hourEST < 0) hourEST += 24;

    Serial.print("Date (EST): ");
    Serial.print(gps.date.day()); Serial.print("/");
    Serial.print(gps.date.month()); Serial.print("/");
    Serial.print(gps.date.year());
    Serial.print(" Time (EST): ");
    Serial.print(hourEST); Serial.print(":");
    Serial.print(gps.time.minute()); Serial.print(":");
    Serial.println(gps.time.second());

    Serial.print("Speed (km/h) = ");
    Serial.println(gps.speed.kmph());

    Serial.print("Altitude (m) = ");
    Serial.println(gps.altitude.meters());

    Serial.print("Satellites = ");
    Serial.println(gps.satellites.value());
}

void displayMPU6050Data() {
    Wire.beginTransmission(MPU_addr1);
    Wire.write(0x3B);  // Starting register for accelerometer
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr1, 6);
    raw_ax = (Wire.read() << 8) | Wire.read();
    raw_ay = (Wire.read() << 8) | Wire.read();
    raw_az = (Wire.read() << 8) | Wire.read();

    xa = raw_ax / 16384.0;
    ya = raw_ay / 16384.0;
    za = raw_az / 16384.0;
    roll = atan2(ya, za) * 180.0 / PI;
    pitch = atan2(-xa, sqrt(ya * ya + za * za)) * 180.0 / PI;

    Wire.beginTransmission(MPU_addr1);
    Wire.write(0x43);  // Starting register for gyroscope
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr1, 6);
    raw_gx = (Wire.read() << 8) | Wire.read();
    raw_gy = (Wire.read() << 8) | Wire.read();
    raw_gz = (Wire.read() << 8) | Wire.read();

    gx = raw_gx / 131.0;
    gy = raw_gy / 131.0;
    gz = raw_gz / 131.0;

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    yawAngle += gz * deltaTime;

    Serial.print("Raw Accelerometer: ax = ");
    Serial.print(raw_ax);
    Serial.print(", ay = ");
    Serial.print(raw_ay);
    Serial.print(", az = ");
    Serial.println(raw_az);

    Serial.print("Calculated Accelerometer (g): ax = ");
    Serial.print(xa, 3);
    Serial.print("g, ay = ");
    Serial.print(ya, 3);
    Serial.print("g, az = ");
    Serial.println(za, 3);

    Serial.print("Raw Gyroscope: gx = ");
    Serial.print(raw_gx);
    Serial.print(", gy = ");
    Serial.print(raw_gy);
    Serial.print(", gz = ");
    Serial.println(raw_gz);

    Serial.print("Calculated Gyro: roll = ");
    Serial.print(roll, 1);
    Serial.print(", pitch = ");
    Serial.print(pitch, 1);
    Serial.print(", yaw = ");
    Serial.println(yawAngle, 1);

    Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
}

