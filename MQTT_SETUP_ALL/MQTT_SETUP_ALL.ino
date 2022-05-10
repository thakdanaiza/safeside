/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cloud-mqtt-broker-sim800l/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon


// Your GPRS credentials, if any
const char apn[] = "Internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "true";
const char gprsPass[] = "true";

// MQTT details
const char* broker = "159.65.221.115";                    // Public IP address or domain name
const char* mqttUsername = "safeside";  // MQTT username
const char* mqttPassword = "safeside_pass_j9SZFA8EIm";  // MQTT password

const char* topicOutput1 = "15335092";


#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#include <PubSubClient.h>

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

#define OUTPUT_1             2

long lastMsg = 0;
uint32_t lastReconnectAttempt = 0;

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

#include <Kalman.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

#include <TinyGPS++.h>

static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

String payload;

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en)
{
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35); // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}

void mqttCallback(char* topic, byte* message, unsigned int len) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < len; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp/output1, you check if the message is either "true" or "false".
  // Changes the output state according to the message
  if (String(topic) == topicOutput1) {
    Serial.print("Changing output to ");
    if (messageTemp == "true") {
      Serial.println("true");
    }
    else if (messageTemp == "false") {
      Serial.println("false");

    }
  }
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker without username and password
  //boolean status = mqtt.connect("GsmClientN");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);

  if (status == false) {
    SerialMon.println(" fail");
    //ESP.restart();
    return false;
  }
  SerialMon.println(" success");
  mqtt.subscribe(topicOutput1);
  return mqtt.connected();
}


void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  Wire.begin(I2C_SDA, I2C_SCL);
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  pinMode(OUTPUT_1, OUTPUT);

  SerialMon.println("Wait...");

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    ESP.restart();
  }
  else {
    SerialMon.println(" OK");
  }

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void loop() {
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  //mqtt.publish("15335092", "19,55,38,13.65364,100.7289,1082,172.77,119.09,169.49,172.59,-0.22,-45.07,-23.77,-0.35,0.15,13.20,");
  if (Serial.available() > 0)
  {
    if (gps.encode(Serial.read()))
    {
      /* Update all the values */
      while (i2cRead(0x3B, i2cData, 14));
      accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
      accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
      accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
      tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
      gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
      gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
      gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

      double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
      timer = micros();

      // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
      // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
      // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(accY, accZ) * RAD_TO_DEG;
      double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
      double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
      double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

      double gyroXrate = gyroX / 131.0; // Convert to deg/s
      double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
      } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
      } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

      gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
      gyroYangle += gyroYrate * dt;
      //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
      //gyroYangle += kalmanY.getRate() * dt;

      compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
      compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

      // Reset the gyro angle when it has drifted too much
      if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
      if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;
      //File file = SD.open("/Data.txt", FILE_APPEND);
      if (gps.time.hour() < 10)payload += "0" ;
      payload += gps.time.hour() ; payload += ",";
      if (gps.time.minute() < 10) payload += "0" ;
      payload += gps.time.minute() ; payload += ",";
      if (gps.time.second() < 10) payload += "0" ;
      payload += gps.time.second() ; payload += ",";
      char bufnewlat[10];
      gcvt(gps.location.lat(), 7, bufnewlat);
      char bufnewlng[10];
      gcvt(gps.location.lng(), 7, bufnewlng);
      payload += bufnewlat ; payload += ",";
      payload += bufnewlng ; payload += ",";
      payload += gps.location.age() ; payload += ",";
      payload += roll ; payload += ",";
      payload += gyroXangle ; payload += ",";
      payload += compAngleX ; payload += ",";
      payload += kalAngleX ; payload += ",";

      payload += pitch ; payload += ",";
      payload += gyroYangle ; payload += ",";
      payload += compAngleY ; payload += ",";
      payload += kalAngleY ; payload += ",";

      payload += gps.speed.kmph() ; payload += ",";
      payload += gps.altitude.meters() ; payload += ",";

      SerialMon.println(payload);
      //file.print(payload);

      double temperature = (double)tempRaw / 340.0 + 36.53;
      //file.println(temperature);
      //Serial.println(x++);
      //client.publish("15335092", payload);
      //client.loop();

      // Length (with one extra character for the null terminator)
      int payload_len = payload.length() + 1;

      // Prepare the character array (the buffer)
      char char_array[payload_len];
      payload.toCharArray(char_array, payload_len);
      mqtt.publish(topicOutput1, char_array);
      SerialMon.println(char_array);

      delay(1000);
    }
    /*
      else
      {
      Serial.println("xx:xx:xx");
      Serial.print(roll); Serial.print("\t");
      Serial.print(gyroXangle); Serial.print("\t");
      Serial.print(compAngleX); Serial.print("\t");
      Serial.print(kalAngleX); Serial.println("\t");

      Serial.print(pitch); Serial.print("\t");
      Serial.print(gyroYangle); Serial.print("\t");
      Serial.print(compAngleY); Serial.print("\t");
      Serial.print(kalAngleY); Serial.println("\t");

      Serial.print(gps.speed.kmph()); Serial.print("\t");
      Serial.print(gps.altitude.meters()); Serial.print("\t");
      Serial.print(gps.satellites.value()); Serial.print("\t");

      double temperature = (double)tempRaw / 340.0 + 36.53;
      Serial.println(temperature);

      }
    */

  }
  if (gps.charsProcessed() < 10)
  {
    SerialMon.println(F("No GPS detected: check wiring."));
    // Length (with one extra character for the null terminator)
    int payload_len = payload.length() + 1;

    // Prepare the character array (the buffer)
    char char_array[payload_len];
    payload.toCharArray(char_array, payload_len);
    mqtt.publish(topicOutput1, char_array);
    delay(1000);
  }
  payload = "";
  mqtt.loop();
}
