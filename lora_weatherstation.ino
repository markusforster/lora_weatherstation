
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>
#include <TTN_esp32.h>
//#include <TTN_CayenneLPP.h>
#include <CayenneLPP.h>

//LoRa
#define UNUSED_PIN 0xFF
#define SCK   5
#define MISO  19
#define MOSI  27
#define SS    18
#define RST   14
#define DIO0  26
#define DIO1  33
#define DIO2  32

//BME280
#define SDA     21
#define SCL     13

//Aneometer
#define ANOPIN  12

const char* devEui = "003B889020C38CB8"; // Change to TTN Device EUI
const char* appEui = "70B3D57ED00319B4"; // Change to TTN Application EUI
const char* appKey = "DB39093BACC4A0DF9B0748120E8B44B8"; // Chaneg to TTN Application Key

TTN_esp32 ttn ;
CayenneLPP lpp(240);

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

float temp = 0.0;
float hum = 0.0;
float pressure = 0.0;

float wind = 0.0;
int wind_count = 0;
const int wind_m_sec = 5;

void mDelay(unsigned long duration) {
  unsigned long st_t = millis();
  while(1) {
    if (millis() - st_t >= duration) {
      return;
    }
  }
}

void startBme() {
  I2Cone.begin(SDA, SCL, 100000);
  bool status_bme = bme.begin(0x76, &I2Cone);
  if (!status_bme) {
    Serial.println("Could not find BME280");
    while(1);
  }
}

void setBMESleep(int deviceAddr) {
  Serial.println("BME280 going to Sleep mode...");
  I2Cone.beginTransmission(deviceAddr);
  I2Cone.write((uint8_t)0xF4);
  I2Cone.write((uint8_t)0b00000000);
  I2Cone.endTransmission();
}

void getBmeReadings() {
  temp = bme.readTemperature();
  hum = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
}

void interrupt_cnt() {
  wind_count++;
}

void measureWind() {
  wind_count = 0;
  attachInterrupt(digitalPinToInterrupt(ANOPIN), interrupt_cnt, RISING);
  mDelay(wind_m_sec * 1000);
  detachInterrupt(digitalPinToInterrupt(ANOPIN));
  wind = ((float)wind_count / (float)wind_m_sec*2.4)/2.0;
}

void sendMeasurements() {

//  StaticJsonBuffer<240> JSONbuffer;
//  JsonObject& JSONencoder = JSONbuffer.createObject();
//
//  JSONencoder["temperature"] = temp;
//  JSONencoder["humidity"] = hum;
//  JSONencoder["pressure"] = pressure;
//  JSONencoder["windspeed"] = wind;
//   
//    
//  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
//
//
//  Serial.println(JSONmessageBuffer);

  CayenneLPP lpp(240);

  lpp.reset();
  lpp.addTemperature(1,temp);
  lpp.addRelativeHumidity(2,hum);
  lpp.addBarometricPressure(3,(float)pressure);
  lpp.addAnalogInput(4,(float)wind);
   
  ttn.sendBytes(lpp.getBuffer(), lpp.getSize());
  mDelay(200);
}

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  Serial.println("Starting device...");

  ttn.begin(SS,UNUSED_PIN,RST,DIO0,DIO1,DIO2);
  ttn.showStatus();
//  ttn.onMessage(message);

  Serial.println("JOINING TTN ");
  ttn.join(devEui,appEui,appKey);
  while (!ttn.isJoined()) {
    Serial.print(".");
    mDelay(1000);
  }
  Serial.println("\njoined.");
  ttn.showStatus();

  measureWind();
  startBme();
  getBmeReadings();
  sendMeasurements();

  esp_sleep_enable_timer_wakeup(60e6);
  setBMESleep(0x77);
  Serial.println("Going to deep sleep....");
  esp_deep_sleep_start();
}

void loop() {
  // put your main code here, to run repeatedly:

}
