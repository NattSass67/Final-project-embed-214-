#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

//for uart
const int baudRate = 9600;
const int uartRxPin = D5;
const int uartTxPin = D4;

SoftwareSerial mySerial(uartRxPin, uartTxPin);
// Replace with your WiFi credentials
const char* ssid = "Jack foot";
const char* password = "1111112784";

// Replace with your MQTT broker IP address
const char* mqttServer = "mqtt-dashboard.com";
const int mqttPort = 1883;

// Replace with your MQTT broker username and password (if required)
const char* mqttUsername = "YourMQTTUsername";
const char* mqttPassword = "YourMQTTPassword";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  mySerial.begin(baudRate);
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Connect to MQTT broker
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  
  while (!client.connected()) {
    if (client.connect("NodeMCUClient")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
  
  // Subscribe to a MQTT topic
  client.subscribe("topic/in");

  // Publish a message
  const char* topic = "topic/out";
  const char* message = "Hello, MQTT!";
  client.publish(topic, message);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); //to process most of all the things about MQTT

  //for uart communication
  if (Serial.available()) {    //after you hit enter on keyboard
    String data = Serial.readStringUntil('\n');   //read from what you type trough keyboard
    Serial.println(data);
    mySerial.println(data);   //sending by using uart like Uart transmit
    
  }
  if (mySerial.available()) { //if data is coming
    String data = mySerial.readStringUntil('\n');  //like Uart recieve
    Serial.println(data);//print to terminal
    client.publish("topic/out",(char*) data.c_str());
    
  }

}

void callback(char* topic, byte* payload, unsigned int length) {
  // Handle incoming MQTT message
  Serial.print("Message received on topic: ");
  Serial.println(topic);
   
   //example payload
  Serial.print("Payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  client.publish("topic/out", "recieved");
  
  // Process the payload data
  
}

void reconnect() {
  // Loop until reconnected to MQTT broker
  while (!client.connected()) {
    if (client.connect("NodeMCUClient")) {
      Serial.println("Connected to MQTT broker");
      client.subscribe("topic/in");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}