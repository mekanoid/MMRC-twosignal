// ------------------------------------------------------------
//
//    MMRC client for controlling two signals
//    Copyright (C) 2019 Peter Kindström
//
//    This program is free software: you can redistribute
//    it and/or modify it under the terms of the GNU General
//    Public License as published by the Free Software 
//    Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
// -----------------------------------------------------------
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>

#include "MMRCsettings.h"

// Wifi initialisation
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// -----------------------------------------------------
// Convert settings in MMRCsettings.h to constants and variables
const char* mqttBrokerIP = IP;
String deviceID = DEVICEID;
String nodeID01 = NODEID01;
String nodeID02 = NODEID02;
String cccCLEES = CLEES;

// -----------------------------------------------------
// Various variable definitions

// Variables for client info
String clientID;      // Id/name for this specific client, shown i MQTT and router

// Define turnout states
int NORMAL = 0;
int REVERSE = 1;
int UP = 3;
int DN = 4;

// Temporary
int tmpCnt = 0;
int tempCnt = 0;
int btnState = 0;     // To get two states from a momentary button

// Select which pin will trigger the configuration portal
// #define TRIGGER_PIN D0

// Uncomment next line to use built in LED on NodeMCU (which is pin D4)
// #define LED_BUILTIN D4

// -----------------------------------------------------
// Topic variables

// Variable for topics to subscribe to
const int nbrSubTopics = 4;
String subTopic[nbrSubTopics];

// Variable for topics to publish to
const int nbrPubTopics = 13;
String pubTopic[nbrPubTopics];
String pubTopicContent[nbrPubTopics];

// Often used topics
String pubTopicSignalOneMain;
String pubTopicSignalOneSlave;
String pubTopicSignalTwoMain;
String pubTopicSignalTwoSlave;
String pubTopicDeviceState;

// -----------------------------------------------------
// Signal ONE variables
int signalOneBright[6];
String signalOneSet[6];
int signalOneLedPin[6];
unsigned long signalOneInterval[6];
unsigned long signalOneMillis[6];
String signalOneBlinkState[6];


// -----------------------------------------------------
// Define which pins to use for different actions - Wemos D1 R2 mini clone
// int btnOnePin = D7;    // Pin for first turnout
// int ledOneUpPin = D2;
// int ledOneDnPin = D3;
// int turnoutOnePin = D4;
// int btnTwoPin = D5;    // Pin for second turnout
// int ledTwoUpPin = D6;
// int ledTwoDnPin = D7;
// int turnoutTwoPin = D8;

/*
 * Standard setup function
 */
void setup() {
  // Setup Arduino IDE serial monitor for "debugging"
  Serial.begin(115200);

  // Define build-in LED pin as output
  signalOneLedPin[1] = D1;
  pinMode(signalOneLedPin[1], OUTPUT);
  
  signalOneLedPin[2] = D2;
  pinMode(signalOneLedPin[2], OUTPUT);

  signalOneLedPin[3] = D3;
  pinMode(signalOneLedPin[3], OUTPUT);
  
  signalOneLedPin[4] = D4;
  pinMode(signalOneLedPin[4], OUTPUT);

  signalOneLedPin[5] = D5;
  pinMode(signalOneLedPin[5], OUTPUT);

  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);

//  pinMode(LED_BUILTIN, OUTPUT);   // For Arduino, Wemos D1 mini
//  pinMode(btnOnePin, INPUT);
//  pinMode(btnTwoPin, INPUT);

//  pinMode(TRIGGER_PIN, INPUT);    // Trigger pin for configuration portal

  // Set initial state
  digitalWrite(signalOneLedPin[1], LOW);
  digitalWrite(signalOneLedPin[2], LOW);
  digitalWrite(signalOneLedPin[3], LOW);
  digitalWrite(signalOneLedPin[4], LOW);
  digitalWrite(signalOneLedPin[5], LOW);
  digitalWrite(D6, LOW);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  // Assemble topics to subscribe and publish to
  if (cccCLEES == "1") {
    deviceID = "clees";
// CLEES support not yet implemented
//    subTopic[0] = deviceID+"/"+cccModule+"/cmd/turnout/"+cccObject;
//    pubTopic[0] = deviceID+"/"+cccModule+"/rpt/turnout"+cccObject;
  } else {
    // Subscribe
    subTopic[0] = "mmrc/"+deviceID+"/"+nodeID01+"/main/set";
    subTopic[1] = "mmrc/"+deviceID+"/"+nodeID02+"/main/set";
    subTopic[2] = "mmrc/"+deviceID+"/"+nodeID01+"/slave/set";
    subTopic[3] = "mmrc/"+deviceID+"/"+nodeID02+"/slave/set";

    // Publish
    pubTopic[0] = "mmrc/"+deviceID+"/$name";
    pubTopicContent[0] = "SJ07 signalkort";

    // The following topic is no longer needed
    pubTopic[1] = "mmrc/"+deviceID+"/$state";
    pubTopicContent[1] = "lost";

    pubTopic[2] = "mmrc/"+deviceID+"/$nodes";
    pubTopicContent[2] = nodeID01+","+nodeID02;
    
    pubTopic[3] = "mmrc/"+deviceID+"/"+nodeID01+"/$name";
    pubTopicContent[3] = "Signal 1";

    pubTopic[4] = "mmrc/"+deviceID+"/"+nodeID01+"/$type";
    pubTopicContent[4] = "Signal control";

    pubTopic[5] = "mmrc/"+deviceID+"/"+nodeID01+"/$properties";
    pubTopicContent[5] = "main,slave";

    pubTopic[6] = "mmrc/"+deviceID+"/"+nodeID01+"/main/$name";
    pubTopicContent[6] = "Huvudsignal";

    pubTopic[7] = "mmrc/"+deviceID+"/"+nodeID01+"/main/$datatype";
    pubTopicContent[7] = "string";

    pubTopic[8] = "mmrc/"+deviceID+"/"+nodeID02+"/$name";
    pubTopicContent[8] = "Signal 2";

    pubTopic[9] = "mmrc/"+deviceID+"/"+nodeID02+"/$type";
    pubTopicContent[9] = "Signal control";

    pubTopic[10] = "mmrc/"+deviceID+"/"+nodeID02+"/$properties";
    pubTopicContent[10] = "main,slave";

    pubTopic[11] = "mmrc/"+deviceID+"/"+nodeID02+"/main/$name";
    pubTopicContent[11] = "Huvudsignal";

    pubTopic[12] = "mmrc/"+deviceID+"/"+nodeID02+"/main/$datatype";
    pubTopicContent[12] = "string";

    // Often used publish topics
    pubTopicSignalOneMain = "mmrc/"+deviceID+"/"+nodeID01+"/main";
    pubTopicSignalTwoMain = "mmrc/"+deviceID+"/"+nodeID02+"/main";
    pubTopicSignalOneSlave = "mmrc/"+deviceID+"/"+nodeID01+"/slave";
    pubTopicSignalTwoSlave = "mmrc/"+deviceID+"/"+nodeID02+"/slave";

    pubTopicDeviceState = pubTopic[1];
    
 }

  // Unique name for this client
  if (cccCLEES == "1") {
    clientID = "CLEES "+deviceID;
  } else {
    clientID = "MMRC "+deviceID;
  }

  // Connect to wifi network
  WiFiManager wifiManager;

  // Start the WifiManager for connection to network
  // First parameter is name of access point, second is the password
  wifiManager.autoConnect("MMRC 2-3 Signal", "1234");

  // Uncomment the next line to reset wifi settings - for testing purposes
  //wifiManager.resetSettings();

  // Connect to MQTT broker and define function to handle callbacks
  client.setServer(mqttBrokerIP, 1883);
  client.setCallback(mqttCallback);

}


/**
 * (Re)connects to MQTT broker and subscribes/publishes to one or more topics
 */
void mqttConnect() {
  char tmpTopic[254];
  char tmpContent[254];
  char tmpID[clientID.length()];
  char* tmpMessage = "lost";
  
  // Convert String to char* for the client.subribe() function to work
  clientID.toCharArray(tmpID, clientID.length()+1);
  pubTopicDeviceState.toCharArray(tmpTopic, pubTopicDeviceState.length()+1);

  // Loop until we're reconnected
  while (!client.connected()) {
  Serial.print("MQTT connection...");

  // Attempt to connect
  // boolean connect (tmpID, pubTopicDeviceState, willQoS, willRetain, willMessage)
  if (client.connect(tmpID,tmpTopic,0,true,tmpMessage)) {
    Serial.println("connected");
    Serial.print("MQTT client id: ");
    Serial.println(tmpID);
    Serial.println("Subscribing to:");

    // Subscribe to all topics
    for (int i=0; i < nbrSubTopics; i++){
      // Convert String to char* for the client.subribe() function to work
      subTopic[i].toCharArray(tmpTopic, subTopic[i].length()+1);

      // ... print topic
      Serial.print(" - ");
      Serial.println(tmpTopic);

      // ... and subscribe to topic
      client.subscribe(tmpTopic);
    }

    // Publish to all topics
    Serial.println("Publishing to:");
    for (int i=0; i < nbrPubTopics; i++){
      // Convert String to char* for the client.publish() function to work
      pubTopic[i].toCharArray(tmpTopic, pubTopic[i].length()+1);
      pubTopicContent[i].toCharArray(tmpContent, pubTopicContent[i].length()+1);

      // ... print topic
      Serial.print(" - ");
      Serial.print(tmpTopic);
      Serial.print(" = ");
      Serial.println(tmpContent);

      // ... and subscribe to topic
      client.publish(tmpTopic, tmpContent);
    }    

  } else {

    // Show why the connection failed
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");

    // Wait 5 seconds before retrying
    delay(5000);
    }
  }

  // Publish new device state = active
  mqttPublish(pubTopicDeviceState, "ready", 1);
  Serial.println("---");

}

/**
 * Function to handle MQTT messages sent to this device
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Don't know why this have to be done :-(
  payload[length] = '\0';

  // Print the topic
  Serial.print("Incoming: ");
  Serial.print(topic);
  Serial.print(" = ");

  // Make strings and print payload
  String msg = String((char*)payload);
  String tpc = String((char*)topic);
  Serial.println(msg);

  // Check set command for main signal ONE
  if (tpc == subTopic[0]){

    // Check för message "Stopp"
    if (msg == "stop") {
      signalOneSet[1] = "off";
      signalOneSet[2] = "on";
      signalOneSet[3] = "off";
      signalOneSet[4] = "off";
      signalOneSet[5] = "off";
      signalOneInterval[1] = 10;
      signalOneInterval[2] = 10;
      signalOneInterval[3] = 10;
      mqttPublish(pubTopicSignalOneMain, "stop", 1);
    }

    // Check för message "Kör 80"
    if (msg == "k80") {
      signalOneSet[1] = "on";
      signalOneSet[2] = "off";
      signalOneSet[3] = "off";
      signalOneSet[4] = "off";
      signalOneSet[5] = "off";
      signalOneInterval[1] = 10;
      signalOneInterval[2] = 10;
      signalOneInterval[3] = 10;
      mqttPublish(pubTopicSignalOneMain, "k80", 1);
    }

    // Check för message "Kör 40"
    if (msg == "k40") {
      signalOneSet[1] = "on";
      signalOneSet[2] = "off";
      signalOneSet[3] = "off";
      signalOneSet[4] = "off";
      signalOneSet[5] = "off";
      signalOneInterval[1] = 10;
      signalOneInterval[2] = 10;
      signalOneInterval[3] = 10;
      mqttPublish(pubTopicSignalOneMain, "k40", 1);
    }
    // Check för message "Kör 40, varsamhet"
    if (msg == "k40varsam") {
      signalOneSet[1] = "on";
      signalOneSet[2] = "off";
      signalOneSet[3] = "on";
      signalOneSet[4] = "off";
      signalOneSet[5] = "off";
      signalOneInterval[1] = 10;
      signalOneInterval[2] = 10;
      signalOneInterval[3] = 10;
      mqttPublish(pubTopicSignalOneMain, "k40varsam", 1);
    }

    // Check för message "Kör 40, kort väg"
    if (msg == "k40kort") {
      signalOneSet[1] = "on";
      signalOneSet[2] = "off";
      signalOneSet[3] = "on";
      signalOneSet[4] = "off";
      signalOneSet[5] = "on";
      signalOneInterval[1] = 10;
      signalOneInterval[2] = 10;
      signalOneInterval[3] = 10;
      signalOneInterval[4] = 10;
      signalOneInterval[5] = 10;
      mqttPublish(pubTopicSignalOneMain, "k40kort", 1);
    }

    // Check för message "Kör på sikt"
    if (msg == "k40sikt") {
      signalOneSet[1] = "off";
      signalOneSet[2] = "blink";
      signalOneSet[3] = "off";
      signalOneSet[4] = "off";
      signalOneSet[5] = "off";
      signalOneInterval[1] = 10;
      signalOneInterval[2] = 10;
      signalOneInterval[3] = 10;
      mqttPublish(pubTopicSignalOneMain, "k40sikt", 1);
    }

  }

  // Check set command for slave signal ONE
  if (tpc == subTopic[2]){

    // Check för message "Vänta Kör 80"
    if (msg == "vk80") {
      signalOneSet[1] = "on";
      signalOneSet[2] = "off";
      signalOneSet[3] = "off";
      signalOneSet[4] = "blink";
      signalOneSet[5] = "off";
      signalOneInterval[3] = 100;
      signalOneInterval[4] = 100;
      signalOneInterval[5] = 100;
      mqttPublish(pubTopicSignalOneSlave, "vk80", 1);
    }

    // Check för message "Vänta Kör 40"
    if (msg == "vk40") {
      signalOneSet[1] = "on";
      signalOneSet[2] = "off";
      signalOneSet[3] = "blink";
      signalOneSet[4] = "off";
      signalOneSet[5] = "blink";
      signalOneInterval[3] = 100;
      signalOneInterval[4] = 100;
      signalOneInterval[5] = 100;

      signalOneBright[5] = signalOneBright[3];
      signalOneBlinkState[5] = signalOneBlinkState[3];

      mqttPublish(pubTopicSignalOneSlave, "vk40", 1);
    }

    // Check för message "Vänta Stopp"
    if (msg == "vstop") {
      signalOneSet[1] = "on";
      signalOneSet[2] = "off";
      signalOneSet[3] = "blink";
      signalOneSet[4] = "off";
      signalOneSet[5] = "off";
      signalOneInterval[3] = 100;
      signalOneInterval[4] = 100;
      signalOneInterval[5] = 100;
      mqttPublish(pubTopicSignalOneSlave, "vstop", 1);
    }
    
}
  
  // Check set command for turnout TWO
  if (tpc == subTopic[1]){

    // Check message
    if (msg == "normal") {
//      turnoutTwoStart();
    }
    if (msg == "reverse") {
//      turnoutTwoStart();
    }
  }

}


/**
 * Publish messages to MQTT broker 
 */
void mqttPublish(String pbTopic, String pbPayload, boolean retained) {

  // Convert String to char* for the client.publish() function to work
  char msg[pbPayload.length()+1];
  pbPayload.toCharArray(msg, pbPayload.length()+1);
  char tpc[pbTopic.length()+1];
  pbTopic.toCharArray(tpc, pbTopic.length()+1);

  // Report back to pubTopic[]
  client.publish(tpc, msg, retained);

  // Print information
  Serial.print("Publish: ");
  Serial.print(pbTopic);
  Serial.print(" = ");
  Serial.println(pbPayload);

}


/**
 * signalOneSet
 */
void signalOneChange(String state, int LEDnr) {

  Serial.print("Signal state = ");
  Serial.print(state);
  Serial.print(", LED no = ");
  Serial.print(LEDnr);
  Serial.print(", LED pin = ");
  Serial.print(signalOneLedPin[LEDnr]);
  

  if (state == "on") {
    // Dim LED up a little bit
    signalOneBright[LEDnr] = signalOneBright[LEDnr]+30;

    // Debug
    Serial.print(", brightness = ");
    Serial.println(signalOneBright[LEDnr]);

    // How fast the LED should be turned on
    signalOneInterval[LEDnr] = 15;

    // Check if we reached full brightness
    if (signalOneBright[LEDnr] >= 254) {
      signalOneBright[LEDnr] = 254;
      signalOneSet[LEDnr] = "";
      signalOneInterval[LEDnr] = 100;

      // Debug
      Serial.print("SignalOne, LED ");
      Serial.print(LEDnr);
      Serial.println(" - ON");

    }
  }

  if (state == "off") {
    // Dim LED down a little bit
    signalOneBright[LEDnr] = signalOneBright[LEDnr]-30;

    // Debug
    Serial.print(", brightness = ");
    Serial.println(signalOneBright[LEDnr]);

    // How fast the LED should be turned off
    signalOneInterval[LEDnr] = 10;

    // Check if we reached zero brightness (=turned off)
    if (signalOneBright[LEDnr] <= 0) {
      signalOneBright[LEDnr] = 0;
      signalOneSet[LEDnr] = "";

    // Debug
    Serial.print("SignalOne, LED ");
    Serial.print(LEDnr);
    Serial.println(" - OFF");
    }
  }

  // Set LED brightness
  analogWrite(signalOneLedPin[LEDnr], signalOneBright[LEDnr]);

}

/**
 * Signal ONE blink LED
 */

void signalOneBlink(int LEDnr) {

    // Debug
//    Serial.println("");
//    Serial.println("## Blink");
//    Serial.println(signalOneBlinkState[LEDnr]);
//Serial.println(signalOneBright[LEDnr]);
//    Serial.println(signalOneInterval[LEDnr]);
//    Serial.println(signalOneLedPin[LEDnr]);
//    Serial.println(signalOneBlinkState[LEDnr]);

  // Check if LED is dimming up
  if (signalOneBlinkState[LEDnr] == "up") {

    // Dim LED down a little bit
    signalOneBright[LEDnr] = signalOneBright[LEDnr]+30;

    // Check if we reached full brightness
    if (signalOneBright[LEDnr] >= 254) {
      signalOneBright[LEDnr] = 254;
      signalOneBlinkState[LEDnr] = "normal";
    }

    // Set LED brightness
    analogWrite(signalOneLedPin[LEDnr], signalOneBright[LEDnr]);

    // Set new blink interval
    signalOneInterval[LEDnr] = 10;

  // ..or if LED is dimming down
  } else if (signalOneBlinkState[LEDnr] == "dn") {

    // Dim LED up a little bit
    signalOneBright[LEDnr] = signalOneBright[LEDnr]-30;

    // Check if we reached zero brightness (=turned off)
    if (signalOneBright[LEDnr] <= 0) {
      signalOneBright[LEDnr] = 0;
      signalOneBlinkState[LEDnr] = "up";

      // Set new blink interval
      signalOneInterval[LEDnr] = 500;
    } else {
      // Set new blink interval
      signalOneInterval[LEDnr] = 10;
    }

    // Set LED brightness
    analogWrite(signalOneLedPin[LEDnr], signalOneBright[LEDnr]);

  // ..else LED is ON
  } else {
    digitalWrite(signalOneLedPin[LEDnr], HIGH);
    signalOneBlinkState[LEDnr] = "dn";
    signalOneInterval[LEDnr] = 100;

    // Debug
//    Serial.print("Led ONE blink - ");
//    Serial.println(signalOneBlinkState[LEDnr]);
  }


}


/**
 * Main loop
 */
void loop()
{

  unsigned long currentMillis = millis();

  // -----------------------------------------------------
  // -----------------------------------------------------
  // -- Waiting for actions

  // -----------------------------------------------------
  // Check connection to the MQTT broker. If no connection, try to reconnect
  if (!client.connected()) {
    mqttConnect();
  }

  // -----------------------------------------------------
  // Wait for incoming MQTT messages
  client.loop();

/*
  // Trigger configuration portal
  if (digitalRead(TRIGGER_PIN) == LOW) {
    WiFiManager wifiManager;
    if (!wifiManager.startConfigPortal("MMRC 2-2 Turnout")) {

      Serial.println("Failed to connect and hit timeout");
      delay(3000);

      // Reset and try again
      ESP.reset();
      delay(5000);
    }
    Serial.println("Connected...yeey :)");
  }
*/

  // -----------------------------------------------------
  // Check for button ONE press
/*  int btnOnePress = digitalRead(btnOnePin);
  if (btnOnePress == LOW) {
    Serial.println("Button ONE pressed");
    delay(400);

    // Start moving turnout ONE
    signalOneSet[1] = "bĺink";
    signalOneInterval[1] = 120;
  }

  // -----------------------------------------------------
  // Check for button TWO press
  int btnTwoPress = digitalRead(btnTwoPin);
  if (btnTwoPress == LOW) {
    Serial.println("Button TWO pressed");
    delay(300);

    // Start moving turnout TWO
    signalOneSet[1] = "off";
    signalOneInterval[1] = 150;
  }
*/

  // -----------------------------------------------------
  // -----------------------------------------------------
  // -- Multitasking actions


  // -----------------------------------------------------
  // Check if it is time to change Signal ONE, Led 1 state
  if(currentMillis - signalOneMillis[1] > signalOneInterval[1] && signalOneSet[1]!="") {

    // Check if we should turn on, turn off or BLINK LED
    if (signalOneSet[1] == "on") {
      signalOneChange("on", 1);
    } else if (signalOneSet[1] == "off") {
      signalOneChange("off", 1);
    } else if (signalOneSet[1] == "bĺink") {
      signalOneBlink(1);
    }

    // Reset LED One millis counter
    signalOneMillis[1] = currentMillis;

  }

  // -----------------------------------------------------
  // Check if it is time to change Signal ONE, Led 1 state
  if(currentMillis - signalOneMillis[2] > signalOneInterval[2] && signalOneSet[2]!="") {

    // Check if we should turn on, turn off or BLINK LED
    if (signalOneSet[2] == "on") {
      signalOneChange("on", 2);
    } else if (signalOneSet[2] == "off") {
      signalOneChange("off", 2);
    } else if (signalOneSet[2] == "bĺink") {
      signalOneBlink(2);
    }

    // Reset LED One millis counter
    signalOneMillis[2] = currentMillis;

  }

  // -----------------------------------------------------
  // Check if it is time to change Signal ONE, Led 1 state
  if(currentMillis - signalOneMillis[3] > signalOneInterval[3] && signalOneSet[3]!="") {

    // Check if we should turn on, turn off or BLINK LED
    if (signalOneSet[3] == "on") {
      signalOneChange("on", 3);
    } else if (signalOneSet[3] == "off") {
      signalOneChange("off", 3);
    } else if (signalOneSet[3] == "blink") {
      signalOneBlink(3);
    }

    // Reset LED One millis counter
    signalOneMillis[3] = currentMillis;

  }

  // -----------------------------------------------------
  // Check if it is time to change Signal ONE, Led 1 state
  if(currentMillis - signalOneMillis[4] > signalOneInterval[4] && signalOneSet[4]!="") {

    // Check if we should turn on, turn off or BLINK LED
    if (signalOneSet[4] == "on") {
      signalOneChange("on", 4);
    } else if (signalOneSet[4] == "off") {
      signalOneChange("off", 4);
    } else if (signalOneSet[4] == "blink") {
      signalOneBlink(4);
    }

    // Reset LED One millis counter
    signalOneMillis[4] = currentMillis;

  }

  // -----------------------------------------------------
  // Check if it is time to change Signal ONE, Led 1 state
  if(currentMillis - signalOneMillis[5] > signalOneInterval[5] && signalOneSet[5]!="") {

    // Check if we should turn on, turn off or BLINK LED
    if (signalOneSet[5] == "on") {
      signalOneChange("on", 5);
    } else if (signalOneSet[5] == "off") {
      signalOneChange("off", 5);
    } else if (signalOneSet[5] == "blink") {
      signalOneBlink(5);
    }

    // Reset LED One millis counter
    signalOneMillis[5] = currentMillis;

  }

}
