

/*
  Read NMEA sentences over sERIAL using Ublox module SAM-M8Q, NEO-M8P, etc
  Base on SparkFun_Ublox_Arduino_Library //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
*/

#include "SparkFun_Ublox_Arduino_Library.h"
#include "utilities.h"
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "WiFi.h"
#include "confwifi.h"

SFE_UBLOX_GPS myGPS;

#include <MicroNMEA.h> //https://github.com/stevemarple/MicroNMEA

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// Definir constantes
#define ANCHO_PANTALLA 128 // ancho pantalla OLED
#define ALTO_PANTALLA 64 // alto pantalla OLED

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26
#define BAND 433E6

 
// Objeto de la clase Adafruit_SSD1306
Adafruit_SSD1306 display(ANCHO_PANTALLA, ALTO_PANTALLA, &Wire, -1);

const char* ssid =SSID;

const char* password =PSW;

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.println("Iniciando pantalla OLED");

    Serial.println("SparkFun Ublox Example");

    if (myGPS.begin(Serial1) == false) {
        Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);

       }
     
     // Iniciar pantalla OLED en la dirección 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
  
    Serial.println("No se encuentra la pantalla OLED");
    while (true);
  
  }
   //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Inicializacion OK!");
 // display.setCursor(0,10);
 // Serial.print("Inicializacion OK!");
 // display.display();

 
  delay(2000);
  
 LoRa.setSignalBandwidth(62.5E3);
 LoRa.setSpreadingFactor(9);

// Iniciamos WIFI
 WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.println("Connecting to WiFi..");

  }

  Serial.println("Connected to the WiFi network");

}

  

 
 void loop()
{
    myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

    if (nmea.isValid() == true) {
        long latitude_mdeg = nmea.getLatitude();
        long longitude_mdeg = nmea.getLongitude();

        Serial.print("Latitude (deg): ");
        Serial.println(latitude_mdeg / 1000000., 6);
        Serial.print("Longitude (deg): ");
        Serial.println(longitude_mdeg / 1000000., 6);
    } else {
        Serial.print("No Fix - ");
        Serial.print("Num. satellites: ");
        Serial.println(nmea.getNumSatellites());
    }

    delay(250); //Don't pound too hard on the I2C bus

    // Limpir buffer
  display.clearDisplay();
 
  // Tamaño del texto
  display.setTextSize(1);
  // Color del texto
  display.setTextColor(SSD1306_WHITE);
  // Posición del texto
  display.setCursor(00, 05);
  display.print("EC4A TRACKER ");
  display.setTextSize(1);
  display.print("SATS: ");
  display.print(nmea.getNumSatellites());
  display.println();
  display.setTextSize(1);
  display.println("LAT:");
  display.setTextSize(2);
  display.println(nmea.getLatitude() /1000000., 6);
  display.setTextSize(1);
  display.println("LON:");
  display.setTextSize(2);
  display.println(nmea.getLongitude() / 1000000., 6);
  
     
  // Enviar a pantalla
  display.display();
  delay(2000);

  Serial.println("Sending packet: ");
 
  // send packet Lora
  LoRa.beginPacket();
  LoRa.print("EC4A TRACKER APRS");
  LoRa.print(nmea.getLatitude() /1000000., 6);
  LoRa.print(nmea.getLongitude() / 1000000., 6);
  LoRa.print(nmea.getNumSatellites());

  LoRa.endPacket();
  delay(7000);
  
  }

//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
    //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
    //for sentence cracking
    nmea.process(incoming);
}
