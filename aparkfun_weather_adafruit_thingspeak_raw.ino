
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>

#include <Wire.h> //I2C needed for sensors
#include "SparkFunMPL3115A2.h" //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFunHTU21D.h" //Humidity sensor - Search "SparkFun HTU21D" and install from Library Manager
#include "secrets.h"

MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
const byte STAT_BLUE = 7;
const byte STAT_GREEN = 8;

const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned long api_mtbs = 60000; //mean time between api requests
unsigned long api_lasttime;   //last time api request has been done

float humidity;
float temp_p;
float temp_h;
float pressure;

/* Adafruit */

#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

Adafruit_CC3000_Client adafruitNativeClient;

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.
/* /Adafruit */

#define THINGSPEAK_URL "api.thingspeak.com"
#define THINGSPEAK_PORT_NUMBER 80


void(* resetFunc) (void) = 0; //declare reset function @ address 0

void setup() {
  Serial.begin(115200);

  pinMode(STAT_BLUE, OUTPUT); //Status LED Blue
  pinMode(STAT_GREEN, OUTPUT); //Status LED Green

  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  weatherShieldSetup();
  //wirelessConnect();
  
  delay(1000);
  updateWeatherData();

  api_lasttime = millis() - api_mtbs;
  Serial.println("Weather Shield online!");
}

void loop() {
  //Print readings every second
  if (millis() - api_lasttime > api_mtbs) {
    api_lasttime = millis();
    digitalWrite(STAT_BLUE, HIGH); //Blink stat LED
    if (!wirelessConnectionIsUp()) {
      Serial.println("Wireless connection is down. reconnecting...");
      wirelessStopAndReboot();
      wirelessConnect(); 
    }
 
    updateWeatherData();

    boolean dataReadError = humidity == 998 || temp_p == -999; //Humidty sensor failed to respond
    if (dataReadError) {
      Serial.println("I2C communication to sensors is not working. Check solder connections.");
      weatherShieldSetup();
    } else {
      displayWeatherData();
      writeWeatherDataToThingSpeak();
    }
    digitalWrite(STAT_BLUE, LOW); //Turn off stat LED
  }
}

void weatherShieldSetup() {
  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags
  
  //Configure the humidity sensor
  myHumidity.begin();
}

void updateWeatherData() {
  //Check Humidity Sensor
  humidity = myHumidity.readHumidity();
  temp_h = myHumidity.readTemperature();
  pressure = myPressure.readPressure();
  temp_p = myPressure.readTemp();
}

void displayWeatherData() {
  Serial.print(F("Humidity = "));
  Serial.print(humidity);
  Serial.print(F("%,"));
  
  Serial.print(F(" temp_h = "));
  Serial.print(temp_h, 2);
  Serial.print(F("C,"));
  
  //Check Pressure Sensor
  
  Serial.print(F(" Pressure = "));
  Serial.print(pressure);
  Serial.print(F("Pa,"));
  
  //Check tempf from pressure sensor
  
  Serial.print(F(" temp_p = "));
  Serial.print(temp_p, 2);
  Serial.print(F("C,"));
  
  Serial.println();
}

boolean wirelessConnectionIsUp() {
  return cc3000.checkConnected() && cc3000.checkDHCP();
}

void wirelessStopAndReboot() {
  cc3000.stop();
  cc3000.reboot();
}

void wirelessConnect() {
   /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }
}

void writeWeatherDataToThingSpeak() {
  uint32_t ip = 0;
   // Try looking up the website's IP address
  Serial.print(THINGSPEAK_URL); Serial.print(F(" -> "));
  while (ip == 0) {
    if (!cc3000.getHostByName(THINGSPEAK_URL, &ip)) {
      Serial.println(F("Couldn't resolve!"));
      return;
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
  String tsData = "field1=" + String(temp_p) + "&field2=" + String(temp_h) + "&field3=" + String(pressure)  + "&field4=" + String(humidity);
  Adafruit_CC3000_Client www = cc3000.connectTCP(ip, THINGSPEAK_PORT_NUMBER);
  if (www.connected()) {
    www.fastrprint(F("POST /update HTTP/1.1\n"));

    www.fastrprint(F("Host: ")); www.fastrprint(THINGSPEAK_URL); www.fastrprint(F("\r\n"));
    www.fastrprint(F("Connection: close\r\n"));
    www.fastrprint(F("X-THINGSPEAKAPIKEY: ")); www.fastrprint(SECRET_WRITE_APIKEY); www.fastrprint(F("\r\n"));
    www.fastrprint(F("Content-Type: application/x-www-form-urlencoded\r\n"));
    www.fastrprint(F("Content-Length: "));
    www.print(tsData.length());
    www.fastrprint(F("\r\n\r\n"));

    www.print(tsData);
    long lastRead = millis();
    Serial.println(F("BEGIN response"));
    while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
    while (www.available()) {
      char c = www.read();
      Serial.print(c);
      lastRead = millis();
    }
  }
  www.close();
  Serial.println();
  Serial.println(F("END response"));
  
  } else {
    Serial.println(F("Connection failed"));    
    return;
  }
  
}

bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}
