// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h>

/*********
  A PlatformIO (Arduino framework) MQTT/CAN demo based on sandeepmistry's arduino CAN library: 
  https://github.com/sandeepmistry/arduino-CAN/tree/master
  AND
  Rui Santos' MQTT tutorial: https://randomnerdtutorials.com  
  GPIOs set for Olimex ESP32-EVB
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Ignore this code, we don't have a BME280 temp sensor uncomment if you do...
//#include <Adafruit_BME280.h>
//#include <Adafruit_Sensor.h>

// Replace the next variables with your SSID/Password combination
const char* ssid = "MYSSID";
const char* password = "mypassword";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "test.mosquitto.org";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

float temperature = 0;
float humidity = 0;
int button1state = 0;

//internal ESP32 temperature sensor doesn't work on ESP32-EVB, abandon!??
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

//OLIMEX ESP32-EVB GPIO pin definitions
const int relay1Pin = 32, button1pin = 34;

void CANtx11bit() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");

  CAN.beginPacket(0x12);
  CAN.write('h');
  CAN.write('e');
  CAN.write('l');
  CAN.write('l');
  CAN.write('o');
  CAN.endPacket();

  Serial.println("CANtx11bit done");

  //delay(1000);
}

void CANtx29bit() {

  // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending extended packet ... ");

  CAN.beginExtendedPacket(0xabcdef);
  CAN.write('w');
  CAN.write('o');
  CAN.write('r');
  CAN.write('l');
  CAN.write('d');
  CAN.endPacket();

  Serial.println("CANtx29bit done");

  //delay(1000);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/relay1, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/relay1") {
    Serial.print("Changing relay1 to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(relay1Pin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(relay1Pin, LOW);
    }
  }
  if(String(topic) == "esp32/CANsend") {
    Serial.print("MQTT CAN Transmission requested...");
    
    //send a CAN frame
    //later, add ability to parse MQTT message to CAN here...
    CANtx11bit();
    
  }
}

//later, add CAN RX listner/MQTT publisher type callback function here...
//REF:  https://github.com/sandeepmistry/arduino-CAN/tree/master/examples/CANReceiverCallback

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client","test","test")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/relay1");
      client.subscribe("esp32/CANsend");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); //wait for serial to come up

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(relay1Pin, OUTPUT);
  pinMode(button1pin, INPUT_PULLUP);

  //setup CAN
  Serial.println("CAN Sender initializing");
  
  //Set RX/TX pins per OLIMEX ESP32-EVB GPIO35/GPIO5:
  CAN.setPins(35, 5);
 
  // start the CAN bus at 250 kbps 
  //warning, the bitrate gets divided by two, something to do with a sleep state bug on newer ESP hardware so multiply by two
  if (!CAN.begin(250E3 * 2)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    // Ignore this code, we don't have a BME280 temp sensor uncomment if you do...
    // Temperature in Celsius
    //temperature = bme.readTemperature();   
    // Uncomment the next line to set temperature in Fahrenheit 
    // (and comment the previous temperature line)
    //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit

    //Read ESP32 internal temperature code (NON-working on this ESP varriant, returns 128)
    //use ESP32 on-chip temperature instead, reportedly degF 
    temperature = temprature_sens_read();

    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);

    // Publish OLIMEX ESP32-EVB:button1 state to button1 topic
    //char butString[8] = "EMPTY";
    button1state = digitalRead(button1pin);
    Serial.print("Button1 is ");
    if(button1state == HIGH){
      char butString[8] = "HIGH";
      Serial.println("HIGH");
      client.publish("esp32/button1", butString);
      }
      else if(button1state == LOW){
        char butString[8] = "LOW";
        Serial.println("LOW");
        client.publish("esp32/button1", butString);
      }


    // ignore BME280 humidity either, uncomment if you do...
    /*humidity = bme.readHumidity();
    
    // Convert the value to a char array
    char humString[8];
    dtostrf(humidity, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    client.publish("esp32/humidity", humString);
    */
  }
}