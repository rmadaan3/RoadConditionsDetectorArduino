#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <FirebaseArduino.h>
#include <FirebaseHttpClient.h>
#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#define WIFI_SSID "iPhone"
#define WIFI_PASSWORD "8800817996"
#define FIREBASE_HOST "iedaarm5-197621.firebaseio.com"
#define FIREBASE_AUTH "vTGvhDCRbfoimAMsDCPBYtHPbW806flT3fIWSW8f"
SoftwareSerial GPS_SoftSerial(D4,D5);/* (Rx, Tx) */
TinyGPSPlus gps;

const uint8_t scl = D6;
const uint8_t sda = D7;
volatile float minutes, seconds;
volatile int degree, secs, mins;
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

    
void setup() 
{
  Serial.begin(115200); /* Define baud rate for serial communication */
  GPS_SoftSerial.begin(9600); /* Define baud rate for software serial communication */
  Wire.begin(sda, scl);
  setupMPU();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) 
      {
        Serial.print(".");
        delay(500);
      }
      Serial.println();
      Serial.print("connected: ");
      Serial.println(WiFi.localIP());
      Firebase.begin(FIREBASE_HOST,FIREBASE_AUTH);
      
}

void loop() 
{
        smartDelay(300);
        unsigned long start;
        double lat_val, lng_val, alt_m_val;
        uint8_t hr_val, min_val, sec_val;
        bool loc_valid, alt_valid, time_valid;
        lat_val = gps.location.lat();
        loc_valid = gps.location.isValid();
        lng_val = gps.location.lng();
        alt_m_val = gps.altitude.meters();  /* Get altitude data in meters */
        alt_valid = gps.altitude.isValid(); /* Check if valid altitude data is available */
        hr_val = gps.time.hour(); /* Get hour */
        min_val = gps.time.minute();  /* Get minutes */
        sec_val = gps.time.second();  /* Get seconds */
        time_valid = gps.time.isValid();  /* Check if valid time data is available */
        if (!loc_valid)
        {          
          Serial.print("Latitude : ");
          Serial.println("*****");
          Serial.print("Longitude : ");
          Serial.println("*****");
        }
        else
        {
          DegMinSec(lat_val);
          Serial.print("Latitude in Decimal Degrees : ");
          Serial.println(lat_val, 6);
          Serial.print("Latitude in Degrees Minutes Seconds : ");
          Serial.print(degree);
          Serial.print("\t");
          Serial.print(mins);
          Serial.print("\t");
          Serial.println(secs);
          DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
          Serial.print("Longitude in Decimal Degrees : ");
          Serial.println(lng_val, 6);
          Serial.print("Longitude in Degrees Minutes Seconds : ");
          Serial.print(degree);
          Serial.print("\t");
          Serial.print(mins);
          Serial.print("\t");
          Serial.println(secs);
        }
        if (!alt_valid)
        {
          Serial.print("Altitude : ");
          Serial.println("*****");
        }
        else
        {
          Serial.print("Altitude : ");
          Serial.println(alt_m_val, 6);    
        }
        if (!time_valid)
        {
          Serial.print("Time : ");
          Serial.println("*****");
        }
        else
        {
          char time_string[32];
          sprintf(time_string, "Time : %02d/%02d/%02d \n", hr_val, min_val, sec_val);
          Serial.print(time_string);    
        }
        recordAccelRegisters();
        gForceX = (gForceX - 0.49)*10;
        printData();
        if (loc_valid && (gForceX<-1.75 || gForceX>1.75))
        {

          String name = Firebase.pushString("logs",String(gForceX,2)+","+String(lat_val,6)+","+String(lng_val,6));
        }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())  /* Encode data read from GPS while data is available on serial port */
      gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)   /* Convert data in decimal degrees into degrees minutes seconds form */
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}
void setupMPU()
    {
      Wire.beginTransmission(0b1101000);
      Wire.write(0x6B);
      Wire.write(0b00000000);
      Wire.endTransmission();  
      Wire.beginTransmission(0b1101000);
      Wire.write(0x1B);
      Wire.write(0x00000000);
      Wire.endTransmission(); 
      Wire.beginTransmission(0b1101000);
      Wire.write(0x1C);
      Wire.write(0b00000000);
      Wire.endTransmission(); 
    }
void recordAccelRegisters() 
    {
      Wire.beginTransmission(0b1101000);
      Wire.write(0x3B);
      Wire.endTransmission();
      Wire.requestFrom(0b1101000,6);
      while(Wire.available() < 6);
      accelX = Wire.read()<<8|Wire.read();
      accelY = Wire.read()<<8|Wire.read();
      accelZ = Wire.read()<<8|Wire.read();
      processAccelData();
    }

    void processAccelData()
    {
      gForceX = accelX / 16384.0;
      gForceY = accelY / 16384.0; 
      gForceZ = accelZ / 16384.0;
    }

    void printData() 
    {
      // Serial.print("Gyro (deg)");
      // Serial.print(" X=");
      // Serial.print(rotX);
      // Serial.print(" Y=");
      // Serial.print(rotY);
      // Serial.print(" Z=");
      // Serial.print(rotZ);
      // Serial.print(" Accel (g)");
      // Serial.print(" X=");*/
      // Serial.print(" ");
      Serial.println(gForceX);
      // Serial.print(" ");
      //erial.print(" Y=");
      //Serial.print(gForceY);
      //Serial.print(" ");
      //Serial.print(" Z=");*/
      //Serial.println(gForceZ);
    }
