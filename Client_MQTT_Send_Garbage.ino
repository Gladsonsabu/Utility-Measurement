#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Alvin";
const char* password = "0568098725";
const char* mqtt_server = "172.10.20.4";
const char* ALL1 = "Node1/Status/ALL";
const char* ALL2 = "Node2/Status/ALL";
const char* mqtt_username = "BR_UTILITY"; // MQTT username
const char* mqtt_password = "UTILITYPASS"; // MQTT password
const char* clientID = "Node1"; // MQTT client ID

unsigned long MQTT_prev_time = 0;
unsigned short MQTT_update_time = 5000;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
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

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientID, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("topic_name");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if((millis() - MQTT_prev_time)>= MQTT_update_time){   
        // client.publish(PWR_topic, String( random(0,100)).c_str());
        // client.publish(Water_topic, String(random(0,100)).c_str());
        // client.publish(LPG_topic, String(random(0,100)).c_str());
        // client.publish(LPG_LEAK, String().c_str());
        client.publish(ALL1, String("|" + String(random(0,100)) + "|" + String(random(0,100)) + "|" + String(random(0,100)) + "|" + String(((random(0,100) > 50)? true:false)) + "|").c_str());
        client.publish(ALL2, String("|" + String(random(0,100)) + "|" + String(random(0,100)) + "|" + String(random(0,100)) + "|" + String(((random(0,100) > 50)? true:false)) + "|").c_str());
                                             // Flush the existing/running moving average filter for Pressure difference
        MQTT_prev_time = millis();
      }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received: ");
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}