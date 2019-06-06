/************************INITIALIZATION******************************************/
/*Import Libraries*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <WiFi101.h>

/*Check WIFI status */
char ssid[] = "Nickfeather";        // Network SSID (name)
char pass[] = "dome";               // Network password
int keyIndex = 0;                   // Network key Index number (needed only for WEP)
int lightStatus = 0;
int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);
#define BNO055_SAMPLERATE_DELAY_MS (50) // Set delay between bno samples
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*Initialize variables*/
int gyroStatus;           // Calibration status of gyro
int accelStatus;          // Calibration status of accel
int magStatus;            // Calibration status of mag
const int turnLim = 7;    // Turning threshold constant
const int stopLim = -7;   // Stopping threshold constant
int leftLight = 9;        // Pin number of left light
int rightLight = 10;      // Pin number of right light
int stopLight = 11;       // Pin number of stop light
int headLight = 12;       // Pin number of headlight
int ON = 255;             // Constant used to turn on LED
int OFF = 0;              // Constant used to turn off LED
int endWifiStatus = 0;

/*************************ARDUINO SETUP**************************************/
void setup(void)
{
  /****Initialize Serial****/
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /****Initialize BNO****/
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  displaySensorDetails();   // Display some basic information on this sensor
  displaySensorStatus();    // Display current status
  bno.setExtCrystalUse(true);

  /****Initialize WiFi****/
  WiFi.setPins(8, 7, 4, 2);             // Configure pins for Adafruit ATWINC1500 Feather
  Serial.println("Access Point Web Server");
  pinMode(led, OUTPUT);                 // Set the LED pin mode
  if (WiFi.status() == WL_NO_SHIELD) {  // Check for the presence of the shield:
    Serial.println("WiFi shield not present");
    while (true); // don't continue
  }
  Serial.print("Creating access point named: "); // print the network name (SSID);
  Serial.println(ssid);
  status = WiFi.beginAP(ssid);                  // Create open network.
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    while (true);                               // don't continue
  }
  delay(10000);           // wait 10 seconds for connection:
  server.begin();         // start the web server on port 80
  printWiFiStatus();      // you're connected now, so print out the status
  Serial.println("\nChecking Wifi");
}

/****************************ARDUINO LOOP************************************/
void loop(void)
{
  /* Collect Sensor Data */
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> lineacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  /* Check Stop */
  if ((float)lineacc.y() < stopLim) {
    analogWrite(stopLight, ON);
    //delay(200);
  } else {
    analogWrite(stopLight, OFF);
  }

  /* Check Turn */
  int turnVal = (int)event.orientation.y;
  if (turnVal < -turnLim) {
    analogWrite(leftLight, ON);
  } else {
    analogWrite(leftLight, OFF);
  }
  if (turnVal > turnLim) {
    analogWrite(rightLight, ON);
  } else {
    analogWrite(rightLight, OFF);
  }

  /* Check WiFi */
  checkWifi();

  delay(BNO055_SAMPLERATE_DELAY_MS); /* Wait the specified delay before requesting next data */
}



/*************************SUBROUTINES*************************************/
/*************************Display Sensor Details**************************/
void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/************Display Sensor Status*******************************/
void displaySensorStatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/*****************Print WiFi Status******************************/
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}

/******************Print Mac Address**************************/
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

/***********************Checking Button Press*********************************/
void checkWifi() {
  if (status != WiFi.status()) { // compare the previous status to the current status
    // it has changed update the variable
    status = WiFi.status();
    if (status == WL_AP_CONNECTED) {       // a device has connected to the AP
      byte remoteMac[6];
      Serial.print("Device connected to AP, MAC address: ");
      WiFi.APClientMacAddress(remoteMac);
      printMacAddress(remoteMac);
    } else { // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<title> Longboard Headlights </title>");
            client.print("<h2> Longboard Headlights </h2>");
            client.print("A MECH 848 Project<br><br>");
            client.print("<a href=\"/H\"><button>LED On</button></a>");
            client.print("<a href=\"/L\"><button>LED Off</button></a>");
            client.print("<a href=\"/W\"><button>Toggle Headlights</button></a>");
            client.print("<br>N. Seale, 2018<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check the client request
        if (currentLine.endsWith("GET /H")) { // GET /H turns the LED on
          digitalWrite(led, HIGH);
        }
        if (currentLine.endsWith("GET /L")) { // GET /L turns the LED off
          digitalWrite(led, LOW);
        }
        if (currentLine.endsWith("GET /W")) {// GET /W toggles the headlights
          if (lightStatus == 1) {
            analogWrite(headLight, OFF);
            lightStatus = 0;
          } else {
            analogWrite(headLight, ON);
            lightStatus = 1;
          }
        }
      }
    }
    client.stop();     // close the connection:
    Serial.println("client disconnected");
  }
}

/**************Continuous Printing Over WiFi***************************/
void checkWifi2() {
  if (status != WiFi.status()) { // compare the previous status to the current status
    // it has changed update the variable
    status = WiFi.status();
    if (status == WL_AP_CONNECTED) {       // a device has connected to the AP
      byte remoteMac[6];
      Serial.print("Device connected to AP, MAC address: ");
      WiFi.APClientMacAddress(remoteMac);
      printMacAddress(remoteMac);
    } else { // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            while (1) {
              sensors_event_t event;
              bno.getEvent(&event);
              imu::Vector<3> lineacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
              String stringOrient = (String)millis() + "\tX: " + (String)event.orientation.x + 
              "\tY: " + (String)event.orientation.y + "\tZ: " + (String)event.orientation.z + "\tAx: ";
              String stringAccel = (String)lineacc.x() + "\tAy: " + (String)lineacc.y() + "\tAz: " + (String)lineacc.z();

              client.print(stringOrient + stringAccel);
              client.print("<br>");
              //delay(1000);

              // Check for stopping
              if ((float)lineacc.y() < stopLim) {
                analogWrite(stopLight, ON);
                //delay(200);
              } else {
                analogWrite(stopLight, OFF);
              }

              //Check for Left/Right tilt
              int turnVal = (int)event.orientation.y;
              if (turnVal < -turnLim) {
                analogWrite(leftLight, ON);
              } else {
                analogWrite(leftLight, OFF);
              }
              if (turnVal > turnLim) {
                analogWrite(rightLight, ON);
              } else {
                analogWrite(rightLight, OFF);
              }

            }
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    client.stop();     // close the connection:
    Serial.println("client disconnected");
  }
}
