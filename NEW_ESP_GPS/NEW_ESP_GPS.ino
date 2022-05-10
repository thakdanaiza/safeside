
// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

#include <TinyGPS++.h> //https://github.com/mikalhart/TinyGPSPlus

#define BLYNK_PRINT Serial
#define BLYNK_HEARTBEAT 30
#define TINY_GSM_MODEM_SIM800

#include <TinyGsmClient.h> // https://github.com/vshymanskyy/TinyGSM
#include <BlynkSimpleSIM800.h> //https://github.com/blynkkk/blynk-library

#include <Wire.h>

// Variables for storing GPS Data
float latitude;
float longitude;

// Set serial for GPS Module
#define SerialMon Serial

// Hardware Serial for builtin GSM Module
#define SerialAT Serial1

const char apn[]  = "Internet";
const char user[] = "true";
const char pass[] = "true";

//static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

//SoftwareSerial ss(RXPin, TXPin);
BlynkTimer timer;

TinyGsm modem(SerialAT);

void setup()
{
  // Set console baud rate
  Serial.begin(9600);
  delay(10);

  // Set-up modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print(F("Connecting to APN: "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");
  //  ss.begin(GPSBaud);
  timer.setInterval(5000L, checkGPS);

}

void checkGPS()
{
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    //Blynk.virtualWrite(V4, "GPS ERROR");
  }
}

void loop()
{
  while (Serial.available() > 0)
  {
    if (gps.encode(Serial.read()))
      displayInfo();
  }

  timer.run();
}

void displayInfo()
{

  if (gps.location.isValid() )
  {

    Serial.print("LAT:  ");
    Serial.println(latitude, 6);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 6);

  }


  Serial.println();
}
