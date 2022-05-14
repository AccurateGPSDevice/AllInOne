#include <SoftwareSerial.h>
#include "Kalman.h"
#include "TinyGPSPlus.h"

static const int RXPin = 4, TXPin = 3;                                      // RX-Pin(4) || TX-Pin(3)
static const uint32_t GPSBaud = 9600;                                       // GPS Baurate 9600

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

long then = 0;

void setup()
{

  Serial.begin(9600);                                                     // Main Baudrate
  ss.begin(GPSBaud);                                                        // GPS Baudrate

  Serial.println(F("Written by Emma\n"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("TinyGPSPlus library version --->>> "));
  Serial.println(TinyGPSPlus::libraryVersion());

  IMU::init();
  IMU::read();

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // Call function for reading GPS data & Compare IMU data
  GPSandIMU_Data();
}


void GPSandIMU_Data()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)                                                // While baudrate for GPS is available Process is continius
    if (gps.encode(ss.read()))                                              // If Arduino read data from GPS

      /*
        Here we display all data from GPS to Serial Monitor
        Data like Location lattitude and longitude
        And also use Date and Time for displaying Real-Time work
      */
      if (gps.location.isValid())                                             // If LOCATION data from GPS available
      {
        Serial.print(gps.location.lat(), 6);                                  // Display location
        Serial.print(" ; ");                                                 // lattitude and longitude
        Serial.print(gps.location.lng(), 6);
        Serial.print(" ; ");


        IMU::read();

        Serial.print(IMU::getRoll());                                             // Print Kalman filtered X coordinate
        Serial.print(" ; ");
        Serial.print(IMU::getRawAccelX());                                        // Print Acc Raw X coordinate

        Serial.print(" ; ");
        Serial.print(IMU::getPitch());                                            // Print Kalman filtered Y coordinate
        Serial.print(" ; ");
        Serial.println(IMU::getRawAccelY());                                      // Print Acc Raw Y coordinate
      }
  //Condition for 
  if (millis() > 5000 && gps.charsProcessed() < 10)                           // If GPS unavailable
  {
    Serial.println(F("No GPS detected: check wiring."));                      // Display "No GPS detected: check wiring." data to Serial Monitor
    while (true);                                                             // While GPS unavailable stay on this condition
  }
}
