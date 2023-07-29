/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Anchor Chain Remote Control / Chain Counter with WLAN.
// Version 1.0, 14.02.2020, AK-Homberger

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#include "index_html.h"              // Web site information for Gauge / Buttons
#include "settings.h"

#define HASTFT
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 135
#define ENABLE_DEMO 0                // Set to 1 to enable Demo Mode with up/down counter, set to 2 for pulsing on the fake counter pin, simulating a rotating gipsy
#define SAFETY_STOP 3                // Defines safety stop for chain up. Stops defined number of events before reaching zero
#define MAX_CHAIN_LENGTH 80          // Define maximum chan length. Relay off after the value is reached
#define TARGET_INCREMENT 5           // amount to alter the desiredlength in meters per keypress
#define WATCHDOG_TIME 1500           // time-out im ms for inactivity of windlass & Web connection

// Wifi: Select AP or Client

#define WiFiMode_AP_STA 1            // Defines WiFi Mode 0 -> AP (with IP:192.168.4.1 and  1 -> Station (client with IP: via DHCP)
const char *ssid = WIFI_SSID;        // Set WLAN name
const char *password = WIFI_PASS;    // WIFI_PASSWORD;  // Set password

WebServer server(80);                // Web Server at port 80
Preferences preferences;             // Nonvolatile storage on ESP32 - To store ChainCounter

// Chain Counter

#define Chain_Calibration_Value 0.33 // Translates counter impuls to meter 0,33 m per pulse
unsigned long Last_int_time = 0;     // Time of last interrupt
unsigned long Last_event_time = 0;   // Time of last event for engine watchdog
int ChainCounter = 0;                // Counter for chain events
int LastSavedCounter = 0;            // Stores last ChainCounter value to allow storage to nonvolatile storage in case of value changes
int desiredLength = 0;
int targetLength = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // To lock/unlock interrupt

// IO Pins
#define Chain_Counter_Pin 27         // Counter impulse is measured as interrupt on this GPIO pin 25
#define Chain_Up_Pin 13              // GPIO pin 14 for Chain Up Relay
#define Chain_Down_Pin 12            // GPIO pin 12 for Chain Down Relay
#define Fake_Counter_Pin 17          // Fake pulse for demo/debug purposes
#define Chain_Goes_Up 2
#define Chain_Goes_Down 15
#define Display_Pin 26               // display on when down


int UpDown = 1;                      // 1 =  Chain down / count up, -1 = Chain up / count backwards
int OnOff = 0;                       // Relay On/Off - Off = 0, On = 1
unsigned long Watchdog_Timer = 0;    // Watchdog timer to stop relay after 1 second of inactivity e.g. connection loss to client



/****** OTA Stuff
 *
 */
 
 void setupOTA () {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  ArduinoOTA.setPassword(OTA_PASS);

  ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
 }


// return true if the conditions to stop lowering the chain have been reached
//
bool stopCriterionReached () {
  float paid_chain = abs(ChainCounter) * Chain_Calibration_Value;
  bool stop = ((ChainCounter <= SAFETY_STOP) && (UpDown == -1) && (OnOff == 1)) ||     // Safety stop counter reached while chain is going up
         ((UpDown == 1) && paid_chain >= MAX_CHAIN_LENGTH) ||                          // No more chain than the MAX
         ((UpDown == 1 && desiredLength > 0) && 
         (paid_chain >= desiredLength) && (paid_chain < desiredLength + Chain_Calibration_Value)
         );        // stop when desired length is reached
  
  return stop;
}

// Chain Event Interrupt
// Enters on falling edge
//=======================================
void IRAM_ATTR handleInterrupt() {

  if (millis() > Last_int_time + 250) {  // Debouncing. No new events for 10 milliseconds

    portENTER_CRITICAL_ISR(&mux);

#if ENABLE_DEMO == 0
    if (digitalRead (Chain_Goes_Up) == LOW) {
      ChainCounter--;   // up
    } else {
      ChainCounter++;   // down in all other cases
    }
 #else   
    ChainCounter += UpDown;             // Chain event: Count up/down
 #endif
 
   if (stopCriterionReached()) {
      digitalWrite(Chain_Up_Pin, LOW );
      digitalWrite(Chain_Down_Pin, LOW );
      OnOff = 0;
    }
    Last_event_time = millis();         // Store last event time to detect blocking chain

    portEXIT_CRITICAL_ISR(&mux);
  }
  Last_int_time = millis();             // Store last interrupt time for debouncing
}

#ifdef HASTFT
TFT_eSPI tft = TFT_eSPI(SCREEN_HEIGHT, SCREEN_WIDTH);

void clearRow (uint8_t row) {
  /// Clear a text row on an Adafruit graphics display
  tft.fillRect(0, tft.fontHeight() * row, SCREEN_WIDTH, tft.fontHeight(), TFT_BLACK); 
}

void printText (uint8_t row, String line, uint16_t color = TFT_GREEN) {   
    //clearRow (row);
    tft.setTextColor(color);
    tft.setTextSize(2);
    tft.setCursor (0,tft.fontHeight() * row);   
    tft.printf ("%s\n", line.c_str());
}

void printIndicator (uint8_t row, uint8_t col, char c, boolean state, uint16_t en_color = TFT_CYAN) {
    if (state) { 
      tft.setTextColor(en_color);
    } else {
      tft.setTextColor(TFT_DARKGREY);
    }
    tft.setTextSize(2);
    tft.setCursor (col * tft.textWidth ("W"),tft.fontHeight() * row);   
    tft.printf ("%c", c);
}


double prevValue[20] = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
void printLine (uint8_t row, String line, double value, String unit="", uint16_t color = TFT_GREEN) {
    if (abs (value - prevValue[row]) < 0.01) return;
    clearRow (row);
    tft.setTextColor(color);
    tft.setTextSize(2);
    tft.setCursor (0,tft.fontHeight() * row);
     if (!unit.equals("")) {
        tft.printf ("%s %.1f%s\n", line.c_str(), value, unit.c_str());
     } else {
        tft.printf ("%s %.1f\n", line.c_str(), value);
     
     }
     prevValue[row] = value;
}
#endif



void Event_Up() {                          // Handle UP request
  server.send(200, "text/plain", "-1000"); // Send response "-1000" means no  chainlength
  Serial.println("Up");
  digitalWrite(Chain_Up_Pin, HIGH );
  digitalWrite(Chain_Down_Pin, LOW );
  Last_event_time = millis();
  UpDown = -1;
  OnOff = 1;
}


void Event_Down() {                         // Handle Down request
  server.send(200, "text/plain", "-1000");  // Send response "-1000" means no  chainlength
  Serial.println("Down");
  digitalWrite(Chain_Up_Pin, LOW );
  digitalWrite(Chain_Down_Pin, HIGH );
  Last_event_time = millis();
  UpDown = 1;
  OnOff = 1;
}

void Event_Stop() {                         // Handle Stop request
  server.send(200, "text/plain", "-1000");  // Send response "-1000" means no  chainlength
  Serial.printf ("Stop @ %d revs\n",ChainCounter);
  digitalWrite(Chain_Up_Pin, LOW );
  digitalWrite(Chain_Down_Pin, LOW );
  OnOff = 0;
}

void Event_Reset() {                        // Handle reset request to reset counter to 0
  desiredLength = ChainCounter = 0;                         
  server.send(200, "text/plain", "-1000");  // Send response "-1000" means no  chainlength
  Serial.println("Reset");
}

void Event_TUp() {                          // Handle target UP request
  Last_event_time = millis();
  if (desiredLength < (MAX_CHAIN_LENGTH - TARGET_INCREMENT)) desiredLength += TARGET_INCREMENT;
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/plain", String (desiredLength));
  vTaskDelay (700/portTICK_PERIOD_MS);    // delay a bit so the operator sees the needle go up
  Serial.printf("Target Up %d m\n", desiredLength);

}


void Event_TDown() {                         // Handle target Down request
  Last_event_time = millis();
  if (desiredLength >= TARGET_INCREMENT) desiredLength -= TARGET_INCREMENT;
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/plain", String (desiredLength));
  vTaskDelay (700/portTICK_PERIOD_MS); // delay a bit so the operator sees the needle go down
  Serial.printf("Target Down %d m\n", desiredLength);
}

void Event_Index() {                         // If "http://<ip address>/" requested
  server.send(200, "text/html", indexHTML);  // Send Index Website
}


void Event_js() {                            // If "http://<ip address>/gauge.min.js" requested
  server.send(200, "text/html", gauge);      // Then send gauge.min.js
}



void Event_ChainCount() {                    // If  "http://<ip address>/ADC.txt" requested

  float temp = (ChainCounter * Chain_Calibration_Value); // Chain in meters
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/plain", String (temp));

  Watchdog_Timer = millis();                 // Watchdog timer is set to current uptime


 

#if ENABLE_DEMO == 1                         // Demo Mode - Counts automatically UP/Down every 500 ms

  if (OnOff == 1) ChainCounter += UpDown;


  if (stopCriterionReached()) {  // Maximum chain length reached

    digitalWrite(Chain_Up_Pin, LOW );
    digitalWrite(Chain_Down_Pin, LOW );
    OnOff = 0;
  }
  Last_event_time = millis();
#endif
}

void handleNotFound() {                                           // Unknown request. Send error 404
  server.send(404, "text/plain", "File Not Found\n\n");
}

void setup() {
  int wifi_retry = 0;


#ifdef HASTFT
  tft.init();
  tft.setRotation (3);
  tft.fillScreen (TFT_BLACK);
    pinMode(4,OUTPUT); // for the screen on/off
#endif

  // Relay output
  pinMode(Chain_Up_Pin, OUTPUT);            // Sets pin as output
  pinMode(Chain_Down_Pin, OUTPUT);          // Sets pin as output
  pinMode(Fake_Counter_Pin, OUTPUT);
  digitalWrite(Chain_Up_Pin, LOW );         // Relay off
  digitalWrite(Chain_Down_Pin, LOW );       // Relay off

  // Init Chain Count measure with interrupt
  pinMode(Chain_Counter_Pin, INPUT_PULLUP); // Sets pin input with pullup resistor
  attachInterrupt(digitalPinToInterrupt(Chain_Counter_Pin), handleInterrupt, FALLING); // Attaches pin to interrupt on falling edge
  
  pinMode(Chain_Goes_Down, INPUT_PULLUP);   // Sets pin input with pullup resistor
  pinMode(Chain_Goes_Up, INPUT_PULLUP);     // Sets pin input with pullup resistor
  pinMode(Display_Pin, INPUT_PULLUP);       // have some pin to light up the display
  #if ENABLE_DEMO != 0 
  pinMode (Display_Pin,INPUT_PULLDOWN);     // fire up the display always
  #endif


  // Init serial
  Serial.begin(115200);
  Serial.print("");
  Serial.println("Start");

  preferences.begin("nvs", false);                  // Open nonvolatile storage (nvs)
  ChainCounter = preferences.getInt("counter", 0);  // Read last saved counter
  //desiredLength = preferences.getInt("target",0);   // target length
  LastSavedCounter = ChainCounter;                  // Initialise last counter value
  //targetLength = desiredLength;
  preferences.end();                                // Close nvs

  WiFi.setHostname ("chaincounter");
  // Init WLAN AP
  if (WiFiMode_AP_STA == 0) {

    WiFi.mode(WIFI_AP);                              // WiFi Mode Access Point
    delay (100);
    WiFi.softAP(ssid, password); // AP name and password

    Serial.println("Start WLAN AP");
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());

  } else {

    Serial.println("Start WLAN Client DHCP");         // WiFi Mode Client with DHCP
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {           // Check connection
      wifi_retry++;
      delay(500);
      Serial.print(".");
      if (wifi_retry > 10) {
        Serial.println("\nReboot");                   // Reboot after 10 connection tries
        ESP.restart();
      }
    }

    setupOTA();

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

  }


  // Handle HTTP request events

  server.on("/", Event_Index);
  server.on("/gauge.min.js", Event_js);
  server.on("/ADC.txt", Event_ChainCount);
  server.on("/up", Event_Up);
  server.on("/down", Event_Down);
  server.on("/stop", Event_Stop);
  server.on("/reset", Event_Reset);
  server.on("/targetup", Event_TUp);
  server.on("/targetdown", Event_TDown);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP Server started");
}


void loop() {
  int wifi_retry = 0;

  server.handleClient();                                           // Handle HTTP requests

  if ( ( millis() > Watchdog_Timer + WATCHDOG_TIME ) ||                     // Check HTTP connnection
       ( (OnOff == 1) && (millis() > Last_event_time + WATCHDOG_TIME)) )  { // Check events if engine is on

    digitalWrite(Chain_Up_Pin, LOW );                              // Relay off after 1 second inactivity
    digitalWrite(Chain_Down_Pin, LOW );
    OnOff = 0;
//   Serial.println ("Stopped due to timeout");
  }

#if ENABLE_DEMO == 2
  // Fake it give a pulse twice per sec.. Ugly I know...
  if ((millis() - Last_event_time) < 250) {
     if (OnOff == 1)  digitalWrite (Fake_Counter_Pin, LOW);  // pulse on the fake counter pin to simulate one
     printIndicator (6,14,'F',false); 
  } else if ((millis()- Last_event_time) > 250) {
     if (OnOff == 1)  digitalWrite (Fake_Counter_Pin, HIGH);  // pulse on the fake counter pin to simulate one 
     printIndicator (6,14,'F',true); 
  } 
  
  if ((millis() - Last_event_time) > 500) {
     if (OnOff == 1)  digitalWrite (Fake_Counter_Pin, LOW);  // pulse on the fake counter pin to simulate one
     printIndicator (6,14,'F',false); 
  } else if ((millis()- Last_event_time) > 750) {
     if (OnOff == 1)  digitalWrite (Fake_Counter_Pin, HIGH);  // pulse on the fake counter pin to simulate one
     printIndicator (6,14,'F',true); 
  }
#elif ENABLE_DEMO == 0
  // reflect the read pins.
   if (digitalRead (Chain_Goes_Up) == LOW) {
    UpDown = -1;          // contrary to the assumed dropping of the anchor (turning gypsy) we signal the anchor is actually going up
  } else {
    UpDown = 1;           // otherwise assume the chain is going down (free fall or using the windlass)
  }
#endif

 
  if (ChainCounter != LastSavedCounter) {                          // Store Chain Counter to nonvolatile storage (if changed)
    preferences.begin("nvs", false);
    preferences.putInt("counter", ChainCounter);
    LastSavedCounter = ChainCounter;
    preferences.end();
  }

 /* 
  if (desiredLength != targetLength) {                            // Store desiredLength to nonvolatile storage (if changed)
    preferences.begin("nvs", false);
    preferences.putInt("target",desiredLength);
    targetLength = desiredLength;
    preferences.end();
  }
*/

  if (WiFiMode_AP_STA == 1) {                                      // Check connection if working as client

    while (WiFi.status() != WL_CONNECTED && wifi_retry < 5 ) {     // Connection lost, 5 tries to reconnect
      wifi_retry++;
      Serial.println("WiFi not connected. Try to reconnect");
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      delay(100);
    }
    if (wifi_retry >= 5) {
      Serial.println("\nReboot");                                  // Did not work -> restart ESP32
      ESP.restart();
    }
  }

  #ifdef HASTFT  

  if (digitalRead (Display_Pin) == LOW) {
    digitalWrite(4,HIGH); // Should force backlight on
    tft.writecommand(ST7789_DISPON);// Switch off the display
    tft.writecommand(ST7789_SLPOUT);// Sleep the display drive
    printLine (0,"Chain Counter  ", (double)WiFi.channel(),"",TFT_SKYBLUE);
    printText (1,"IP: "+WiFi.localIP().toString(),TFT_YELLOW);
    printLine (3,"L",ChainCounter * Chain_Calibration_Value,"m");
    printLine (4,"T",desiredLength,"m",TFT_ORANGE);
    
    printIndicator (6,0,'U',UpDown == -1);
    printIndicator (6,2,'D',UpDown == 1);
    printIndicator (6,4,'E',OnOff == 1);
    printIndicator (6,6,'u',digitalRead (Chain_Goes_Up) == LOW);
    printIndicator (6,8,'d',digitalRead (Chain_Goes_Down) == LOW);
    printIndicator (6,10,'P',digitalRead (Chain_Counter_Pin) == LOW);

  } else {
    digitalWrite(4,LOW); // Should force backlight off
    tft.writecommand(ST7789_DISPOFF);// Switch off the display
    tft.writecommand(ST7789_SLPIN);// Sleep the display drive
  }

#endif

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
 // if ( Serial.available() ) {
 //   Serial.read();
 // }
 
  ArduinoOTA.handle();

  vTaskDelay (50/portTICK_PERIOD_MS);

}
