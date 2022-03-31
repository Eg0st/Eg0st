/*
  Basic ESP8266 MQTT example
  This sketch demonstrates the capabilities of the pubsub library in combination
  with the ESP8266 board/library.
  It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
  It will reconnect to the server if the connection is lost using a blocking
  reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
  achieve the same result without blocking the main loop.
  To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/

#include "WiFi.h"
#include <PubSubClient.h>
#include "DHT.h"

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"


//---------------Wlan Define---------------------
const char* ssid = "WLAN-2400G";
const char* password = "!Gauting-82131-Berg-92";

//---------------MQTT Define---------------------
const char* mqtt_server = "192.168.2.2";
const char* clientid = "d1101";

//---------------WIFI objekt erstellen---------------------
WiFiClient wifiClient;
PubSubClient client(wifiClient);

//---------------Timer init---------------------
unsigned long lastMsg = 0;
#define time_wait 30000

//---------------MQTT Buffer---------------------
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];

//---------------MQTT Topic---------------------
#define topic_BUFFER_SIZE (30)
char topic[topic_BUFFER_SIZE];

//---------------Value of the MQTT payload---------------------
float value = 0;

//---------------LED Pin define---------------------
#define BUILTIN_LED 13



//---------------Define sht---------------------
Adafruit_SHT31 sht31 = Adafruit_SHT31();


//------------------------------------
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}




//------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

//------------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      
      client.publish(clientid, "online");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float dht_temp(){
  float t = sht31.readTemperature();
  return t;
}

float dht_humidity(){
  float h = sht31.readHumidity();
  return h;
}



//------------------------------------
void setup() {
  //---------------Init Serial---------------------
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  
  //---------------Init Wifi---------------------
  setup_wifi();

  //---------------Init MQTT Server---------------------
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);


   //---------------Init sht Sensor---------------------
    Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
}





//------------------------------------
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();



  unsigned long now = millis();
  value = 21.45;
  if (now - lastMsg > time_wait) {
    lastMsg = now;
    snprintf (msg, MSG_BUFFER_SIZE, "%f", dht_temp());
    snprintf (topic, topic_BUFFER_SIZE, "%s/temperatur", clientid);
    Serial.print("Publish message:");
    Serial.println(msg);
    client.publish(topic, msg);

    snprintf (msg, MSG_BUFFER_SIZE, "%f", dht_humidity());
    snprintf (topic, topic_BUFFER_SIZE, "%s/hunidity", clientid);
    Serial.print("Publish message:");
    Serial.println(msg);
    client.publish(topic, msg);
  }
}
