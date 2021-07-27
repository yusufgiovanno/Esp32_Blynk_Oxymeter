/*
 * Dev : Yusuf Giovanno
Sensor & ESP32
--------------
VCC = 3v3
SDA = 22
SCL = 21
GND = GND
--------------

Blynk
----------
SPO2  = V0
HBeat = V1
----------
*/

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#define TIMETOBOOT 3000
#define SCALE 88.0
#define SAMPLING 5
#define FINGER_ON 30000
#define MINIMUM_SPO2 80.0

MAX30105 particleSensor;

char auth[] = "Your   Blynk  Auth";
char ssid[] = "Your   Wifi   SSID";
char pass[] = "Your   Wifi   Password";

double avered = 0; 
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;

double ESpO2 = 95.0;
double FSpO2 = 0.7;
double frate = 0.95;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

#define USEFIFO
void setup(){
  Serial.begin(115200);
  Serial.println("Initializing...");

  Blynk.begin(auth, ssid, pass);
  

  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)){
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
  }

  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();
}

void loop(){
  uint32_t ir, red , green;
  double fred, fir;
  double SpO2 = 0;

#ifdef USEFIFO
  particleSensor.check();

  while (particleSensor.available()){
    #ifdef MAX30105
       red = particleSensor.getFIFORed();
        ir = particleSensor.getFIFOIR();
    #else
        red = particleSensor.getFIFOIR();
        ir = particleSensor.getFIFORed();
    #endif

    long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20){
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
   
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);
    aveir = aveir * frate + (double)ir * (1.0 - frate);
    sumredrms += (fred - avered) * (fred - avered);
    sumirrms += (fir - aveir) * (fir - aveir);
    if ((i % SAMPLING) == 0) {
      if ( millis() > TIMETOBOOT) {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;

        if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
        if ( ir_forGraph < 80.0) ir_forGraph = 80.0;
        if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
        if ( red_forGraph < 80.0 ) red_forGraph = 80.0;

        if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2;
        float temperature = particleSensor.readTemperatureF();
        Blynk.run();
        Blynk.virtualWrite(V0,ESpO2);
        Blynk.virtualWrite(V1,beatAvg);
        Serial.print(" Oxygen % = ");
        Serial.println(ESpO2);
      }
    }
    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      SpO2 = -23.3 * (R - 0.4) + 100;
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;

      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample();

  }
#endif
}
