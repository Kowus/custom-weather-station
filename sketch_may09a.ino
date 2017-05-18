#include "Adafruit_FONA.h"
#include <Wire.h> //I2C needed for sensors
#include "SparkFunMPL3115A2.h" //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFunHTU21D.h" //Humidity sensor - Search "SparkFun HTU21D" and install from Library Manager
#include <ArduinoJson.h>


#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
const byte STAT_BLUE = 13;
const byte STAT_GREEN = 8;
const byte STAT_RED = 10;

const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WSPEED = 3;
const byte RAIN = 2;

const byte WDIR = A0;

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned long lastSecond = 0;; //The millis counter to see when a second rolls by
char output[250];
int myState = HIGH;
bool error = true;
void setup()
{
  pinMode(STAT_BLUE, OUTPUT); //Status LED Blue
  pinMode(STAT_GREEN, OUTPUT); //Status LED Green
  pinMode(STAT_RED, OUTPUT); //Status LED RED
  Serial.begin(115200);
  Serial.println("Weather Shield Booting");
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1) {
      digitalWrite(STAT_RED, myState);
      myState = !myState;
      delay(100);
    }
  }
  delay(1000);
  Serial.println("Weather Shield online!");

  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  //Configure the humidity sensor
  myHumidity.begin();
  fona.setGPRSNetworkSettings(F("internet"), F(""), F(""));
  fona.setHTTPSRedirect(true);

  delay(1000);

  Serial.println("Enabling GPRS");
  while (!fona.enableGPRS(true));
  Serial.println("GPRS Enabled");
  delay(1000);
  Serial.println(F("Enabling GPS"));
  while (!fona.enableGPS(true));
  Serial.println(F("GPS Enabled"));
  //  if (!fona.enableNetworkTimeSync(true)){
  //    Serial.println("Failed to enable Network time sync");
  //  }
  //  Serial.println("Network time sync enabled");
  //  if (!fona.enableNTPTimeSync(true, F("pool.ntp.org"))){
  //          Serial.println(F("Failed to enable NTP time sync"));
  //  }
  //  Serial.println("NTP time sync enabled");
  delay(1000);
  pinMode(STAT_BLUE, OUTPUT); //Status LED Blue
  pinMode(STAT_GREEN, OUTPUT); //Status LED Green


  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  delay(1000);
}

void loop()
{

  digitalWrite(STAT_GREEN, HIGH);
  digitalWrite(STAT_BLUE, LOW);
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();



  Serial.println("Beginning Reading");
  //Print readings every second
  //  if (millis() - lastSecond >= interval)  {


  digitalWrite(STAT_BLUE, HIGH); //Blink stat LED
  

  //Check Humidity Sensor
  float humidity = myHumidity.readHumidity();

  if (humidity == 998) //Humidty sensor failed to respond
  {
    Serial.println("I2C communication to sensors is not working. Check solder connections.");

    //Try re-initializing the I2C comm and the sensors
    myPressure.begin();
    myPressure.setModeBarometer();
    myPressure.setOversampleRate(7);
    myPressure.enableEventFlags();
    myHumidity.begin();
  }
  else
  {
    uint16_t vbat;
    float latitude, longitude, speed_kph, heading, speed_mph, altitude;
    char muffer[30] = " ";
    String dat = String(muffer);
    root.set<float>("light", get_light_level());   // Light
    root.set<float>("wind_dir", get_wind_direction()); // Wind Direction
    root.set<float>("wind_spd", 0.00);  // Wind Speed
    root.set<float>("pressure", myPressure.readPressure());    // pressure
    root.set<float>("temperature", myHumidity.readTemperature());      // temperature
    root.set<float>("rain", 0.00);      // rainfall
    root.set<float>("humidity", humidity);      // humidity
    if (! fona.getBattPercent(&vbat)) {
      Serial.println(F("Failed to read Batt"));
    } else {
      root.set<uint16_t>("battery", vbat);
    }
    root.set<float>("latitude", latitude);      // latitude
    root.set<float>("longitude", longitude);      // longitude
    root.set<float>("altitude", altitude);      // altitude
    //      root.set<String>("date", dat);           //date

    root.printTo(output);
    Serial.println(output);

    uint16_t statuscode;
    int16_t length;
    Serial.println("posting data to cloud!");
    char url[80] = "weather-stationgh.herokuapp.com/publish";
    if (!fona.HTTP_POST_start(url, F("application/json"), (uint8_t *) output, strlen(output), &statuscode, (uint16_t *)&length)) {
      Serial.println("Failed!");
    }
    while (length > 0) {
      while (fona.available()) {
        char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
        loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
        UDR0 = c;
#else
        Serial.write(c);
#endif

        length--;
        if (! length || length <= 0) {
          digitalWrite(STAT_BLUE, LOW); //Turn off stat LED
          break;
        }
      }

    }
    Serial.println(F("\n****"));
    fona.HTTP_POST_end();
  }

  digitalWrite(STAT_BLUE, LOW); //Turn off stat LED

  //  }


  //    Serial.println(millis());

  Serial.println("finished logging");

  delay(300000);


}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);

  float lightSensor = analogRead(LIGHT);

  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

  lightSensor = operatingVoltage * lightSensor;

  return (lightSensor);
}


int get_wind_direction()
{
  unsigned int adc;

  adc = analogRead(WDIR); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 380) return (113);
  else if (adc < 393) return (68);
  else if (adc < 414) return (90);
  else if (adc < 456) return (158);
  else if (adc < 508) return (135);
  else if (adc < 551) return (203);
  else if (adc < 615) return (180);
  else if (adc < 680) return (23);
  else if (adc < 746) return (45);
  else if (adc < 801) return (248);
  else if (adc < 833) return (225);
  else if (adc < 878) return (338);
  else if (adc < 913) return (0);
  else if (adc < 940) return (293);
  else if (adc < 967) return (315);
  else if (adc < 990) return (270);
  else return (-1); // error, disconnected?
}




