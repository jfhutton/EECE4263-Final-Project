/*************************************************************************************
 * Sandbox for developing MQTT apps at Lipscomb
 ************************************************************************************/

/*************************************************************************************
 *     Program name: Lab07.cpp
 *          Version: 2.2
 *             Date: Feb 11, 2017
 *           Author: Greg Nordstrom
 *         Modified: John Hutton
 *             Date: Mar 29, 2024
 *         Platform: ESP32
 *    Additional HW: Buzzer, 7-seg LED, 74HC595, rotary encoder w/ button, 220 ohm 
 *                   resistors for 7-seg LED.
 * Additional files: Lab07.h
 *  Req'd libraries: WiFi, PubSubClient, ArduinoJson, Bounce2, Rotary
 *
 * This program implements a simple communication network consisting of up to 16
 * Huzzah32 Feather with an ESP32 processors. Nodes communicate via an MQTT broker
 * (typically a RPi running Mosquitto) to send heartbeat messages as well as "ringring"
 * messages. If a ringring message is received from a valid node (0-16), the
 * receiver plays a brief dual-tone sound using a local buzzer. Nodes must also
 * respond to "topics" messages by sending a "registeredFor" message listing all
 * the topics for which it is registered. These are listed below:
 * 
 * Modified for ESP32 board (fundamentally the same as original ESP8266 design)
 * 
 * Received messages:
 *    Topic: "ece/node01/topics"
 *    Usage: To request a list of all topics this node is registered for
 *  Payload: none 
 *
 * Emitted messages:
 *    Topic: "ece/node01/registeredFor"
 *    Usage: Reply with all N topics this node is registered for
 *  Payload: {"NodeName":"node01", "Topic0":"topic0",...,"TopicN-1":"topicN-1"}
 *
 *    Topic: "ece/node01/ringring"
 *    Usage: Sends a ringring request to other nodes
 *  Payload: {"srcNode":"nodess","dstNode":"nodedd"}
 *           where ss and dd are source and destination node numbers, respectively
 *
 * This program uses the public domain PubSubClient library to perform MQTT
 * messaging functions, and all message payloads are encoded in JSON format.
 * IMPORTANT NOTES ON PubSubClient MQTT library for Arduino:
 * 1. Default message size, including header, is only 128 bytes (rather small).
 * 2. Increase/decrease by changing MQTT_MAX_PACKET_SIZE inside PubSubClient.h.
 * 3. As of 6/14/16, this value is set to 512 (in PubSubClient.h), allowing 7-10
 *    topics to be displayed (depends on individual topic lengths of course).
 * 
 * This code does not use interrupts for pushbutton processing (a fool's errand
 * when the switches are not hardware debounced). Instead, each button is debounced 
 * by a Bounce2 object every 8mS driven by an interrupt-driven timer (Ticker).
 * 
 * This node responds to topics messages by sending a registeredFor message:
 * 
 *    Topic: "ece/node01/registeredFor"
 *    Usage: Reply with all N topics this node is registered for
 *  Payload: {"NodeName":"node01", "Topic0":"topic0",...,"TopicN-1":"topicN-1"}
 *
 * It should be noted that this node's functionality can be fully exercised using
 * an MQTT sniffing program such as MQTT-Spy (avaliable on GitHub). MQTT-Spy is a 
 * Java program, so Java must be installed on the host machine. MQTT-spy is invoked 
 * on a Windows machine from the command line as follows:
 *     
 *     C:>java -jar mqtt-spy-x.x.x-jar-with-dependencies.jar 
 * 
 * where x.x.x is the current version of MQTT.
 * 
 * Modification Notes:
 * - Lab05 had similar modifications...
 * - Pins have been adapted for the ESP32
 * - Libraries have been adapted for the ESP32
 * 
 ************************************************************************************/
// included configuration file and support libraries
#include <Esp.h>            // Esp32 support
#include <WiFi.h>           // wi-fi support
#include <PubSubClient.h>   // MQTT client (by Nick O'Leary?? vTBD version)
#include <ArduinoJson.h>    // for encoding/decoding MQTT payloads in JSON format
                            // by Benoit Blanchon (OLD version vTBD version)
#include <Bounce2.h>        // debounce pushbutton (by Thomas O Frederics vTBD version)
#include <ESP32Encoder.h>   // New libary (in arduino libraries) for rotary encoder
                            // Slightly simpler to implement
                            // https://github.com/madhephaestus/ESP32Encoder
#include "Lab07.h"          // included in this project

WiFiClient wfClient;              // create a wifi client
PubSubClient psClient(wfClient);  // create a pub-sub object (must be
                                  // associated with a wifi client)

// Bounce Button object here

// Global Variables
// May need some character arrays for processing strings and json payloads
char json_Buffer[200];  // May need more than one json buffer
char sbuf[80];          // May need more than one for char* operations

// Rotary encoder variables.
// Consider how you will send the 7-segment bits to the shift array to match the
// Schematic

ESP32Encoder rotaryEncoder;  // Suggest using this library for the shift register

// If you need larger timing in your loop (for things like heartbeat)
unsigned long mscurrent = millis();
unsigned long mslastMQTT = mscurrent;
unsigned long mslastHeartbeat = mscurrent;

/* Prototype functions */
void connectWifi();
void setupMQTT();
void connectMQTT();
void sendHeartbeatMessage();
void registerForTopics();
void processMQTTMessage(char*, byte*, unsigned int);
void BIST();
// will likely have several more functions to keep your code clean

void setup() {
  Serial.begin(115200);
  while(!Serial) {
    // wait for serial connection
    delay(1);
  }
  Serial.println("Serial ready!");

  // Pin and other setup
  pinMode(pdButton, INPUT_PULLUP); // pin 0 corresponds to GPIO0, which corresponds to Pin 19
  // finish button setup

  // initialize buzzer
  pinMode(buzzerPin, OUTPUT); // buzzer ("ringring" message enunciator)

  // 74HC595 pins
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Rotary Setup
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  rotaryEncoder.attachHalfQuad(dtPin, clkPin);
  
  // setup connections
  connectWifi();
  setupMQTT();
  connectMQTT();

  BIST();
  // Flash the on-board LED five times to let user know
  // that the Huzzah32 board has been initialized and ready to go.
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, 0);  // active low
    delay(200);
    digitalWrite(LED_BUILTIN, 1);
    delay(150);
  }
}

void loop() {
  mscurrent = millis();
  // This is largely a reactive program, and as such only uses
  // the main loop to maintain the MQTT broker connection and
  // regularly call the psClient.loop() code to check for new
  // messsages
  psClient.loop();
  // Check buttons
  
  // update rotary encoder object
  

  // reconnect to MQTT server if connection lost
  if ( (mscurrent - mslastMQTT) > MQTT_CONNECT_CHECK) {
    mslastMQTT = mscurrent;
    if (!psClient.connected()) {
      Serial.println("MQTT not connected! Trying reconnect.");
      connectMQTT();
    } 
  }
  // send heartbeat (HEARTBEAT_INTERVAL = 0 means no heartbeat)
  if( ((mscurrent - mslastHeartbeat) > HEARTBEAT_INTERVAL) &&
      HEARTBEAT_INTERVAL != 0 ) {
      mslastHeartbeat = mscurrent;
      sendHeartbeatMessage();
    }
}

/**********************************************************
 * Helper functions
 *********************************************************/
void connectWifi() {
  // attempt to connect to the WiFi network
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print(" network");
  delay(10);
  #ifdef LIPSCOMB
    WiFi.begin(ssid);            // Lipscomb WiFi does NOT require a password
  #elif defined(ETHERNET)
    WiFi.begin(ssid);
  #else
    WiFi.begin(ssid, password);  // For WiFi networks that DO require a password
  #endif

  // advance a "dot-dot-dot" indicator until connected to WiFi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // report to console that WiFi is connected and print IP address
  Serial.print("MAC address = "); Serial.print(WiFi.macAddress());
  Serial.print(", connected as "); Serial.print(WiFi.localIP());
  Serial.println(".");
}
void setupMQTT () {
  // specify MQTT broker's domain name (or IP address) and port number
  Serial.print("Initalizing MQTT object with broker="); Serial.print(mqttBroker);
  Serial.print(" and port="); Serial.print(mqttPort);
  Serial.print("..");
  psClient.setServer(mqttBroker, mqttPort);

  // Specify callback function to process messages from the broker.
  psClient.setCallback(processMQTTMessage);
  Serial.println(".done");
}
void connectMQTT()
{
  // Ping the server before trying to reconnect
  int WiFistatus = WiFi.status();
  if (WiFistatus != WL_CONNECTED) {
    Serial.print("WiFi check failed!  Debug the Wireless connection...");
  } else {
    //Serial.println("passed!");
    // Try to connect to the MQTT broker (let loop() take care of retries)
    Serial.print("Connecting to MQTT with nodeName=");
    Serial.print(nodeName);
    Serial.print(" ... ");
    if (psClient.connect(nodeName)) {
      Serial.println("connected.");
      // clientID (<nodename>) MUST BE UNIQUE for all connected clients
      // can also include username, password if broker requires it
      // (e.g. psClient.connect(clientID, username, password)
      // once connected, register for topics of interest
      registerForTopics();
      sprintf(sbuf,"MQTT initialization complete\r\nReady!\r\n\r\n");
      Serial.print(sbuf);
    }
    else {
      // reconnect failed so print a console message, wait, and try again
      Serial.println(" failed!");
      Serial.print("MQTT client state=");
      Serial.println(psClient.state());
      Serial.print("(Is processor whitelisted?  ");
      Serial.print("MAC=");
      Serial.print(WiFi.macAddress());
      Serial.println(")");
    }
  }
}
void registerForTopics() {
  // register with MQTT broker for topics of interest to this node
  Serial.print("Registering for topics...");
  
  // Register for the topics you need here
  
  Serial.println("Registeration done.");
}
void processMQTTMessage(char* topic, byte* json_payload, unsigned int length) {
  // This code is called whenever a message previously registered for is
  // RECEIVED from the broker. Incoming messages are selected by topic,
  // then the payload is parsed and appropriate action taken. (NB: If only
  // a single message type has been registered for, then there's no need to
  // select on a given message type. In practice this may be rare though...)
  //
  // For info on sending MQTT messages inside the callback handler, see
  // https://github.com/bblanchon/ArduinoJson/wiki/Memory-model.
  //
  // NB: for guidance on creating proper jsonBuffer size for the json object
  // tree, see https://github.com/bblanchon/ArduinoJson/wiki/Memory-model

  // process messages by topic
  // DEBUG:  process messages by topic
  //Serial.print("Topic => ");
  //Serial.println(topic);

  // Parse for topics here
  
}
void sendHeartbeatMessage() {
  // message topic: "ece/node01/heartbeat"
  // payload: NodeName, NodeType

  // heartbeat message here


  Serial.println("Sent heartbeat.");
}
void BIST () {
  // Good to have a hardware check that you can
  // Easily turn on/off from setup

}