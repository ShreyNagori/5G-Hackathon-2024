#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "twilio.hpp"

MAX30105 particleSensor;
#define DS18B20 5 //define PIN No. at which the Temperature sensor is connected
#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
float bodytemperatureC; // variable for calculating body temperature in Celsius
float bodytemperatureF; // variable for calculating body temperature in Fahrenheit

const char* ssid = "Galaxy M42";  // Enter your Wi-Fi SSID here
const char* password = "135798642";  //Enter Wi-Fi Password here

String serverName = "https://api.thingspeak.com/update?api_key=OH3YZO59WS302XOL";

byte pulseLED = 33; //Must be on PWM pin
byte readLED = 32; //Blinks with each data read

OneWire oneWire(DS18B20);
DallasTemperature sensors(&oneWire);

void setup() {
  pinMode(14, OUTPUT); //Three indicator LED pinouts
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:

  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  Serial.println("Connecting to "); // Attempt connecting to WiFi
  Serial.println(ssid);
 
  //connect to your local wi-fi network
  WiFi.begin(ssid, password);
 
  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());   //Local IP Address is printed

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { //Use default I2C port, 400kHz speed
    Serial.println(F("MAX30105 was not found. Please check wiring/power.")); //Attempt to initialize MAX30102
    while (1);
  }

  Serial.println(F("Place your finger on the sensor. Press any key to start conversion"));
  while (Serial.available() == 0); //wait until user presses a key
  Serial.read();
  
  // setting the default parameters for the IR and read light
  byte ledBrightness = 0x7f; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  sensors.begin();
}

void loop() {
  HTTPClient http;
  bufferLength = 100; 
  sensors.requestTemperatures(); // Get temperature readings from the DS18B20
  bodytemperatureC = sensors.getTempCByIndex(0);
  bodytemperatureF = (bodytemperatureC * 9.0 / 5.0)+40.0;

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++) {
    while (particleSensor.available() == false) //do we have new data? 
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102. Heart rate and SpO2 are calculated every 1 second
  while (1) {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++) {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); 

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    Serial.print(F("HR=")); //print Heart Rate
    Serial.print(heartRate/2 + 40, DEC);
    Serial.print(F(", HRvalid=")); // 1 if the Heart rate is valid
    Serial.print(validHeartRate, DEC);     
    Serial.print(F(", SPO2=")); //print SP02
    Serial.print(spo2, DEC);
    Serial.print(F(", SPO2Valid=")); //1 if the calculated SP02 is valid
    Serial.println(validSPO2, DEC);
    Serial.print("Body Temperature: "); // prints the Body temperature
    Serial.print(bodytemperatureF);
    Serial.println("°F");

    if (validHeartRate && validSPO2) {
      String url = serverName + "&field1=" + heartRate/2 + "&field2=" + spo2 + "&field3=" + bodytemperatureF ; // Define our entire url
      http.begin(url.c_str()); 
      int httpResponseCode = http.GET(); 
      if (httpResponseCode > 0) { 
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
      } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    }

    delay(30000); // Delay for 1 minute (60,000 milliseconds)
  } 
}






