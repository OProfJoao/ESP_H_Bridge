//!---------------------       Inclusões de bibliotecas     ---------------------
#include <Arduino.h>
#include <WiFi.h>
#include "PubSubClient.h"
#include "env.h"
#include <WiFiClientSecure.h>

//!---------------------       Definição dos pinos      ---------------------
#define FORWARD_DIRECTION_PIN 32 //* Forward Direction
#define BACKWARD_DIRECTION_PIN 33 //* Backward Direction

#define STATUS_LED_R 14
#define STATUS_LED_G 27
#define STATUS_LED_B 26

#define PWM_FORWARD 0
#define PWM_BACKWARD 1
#define PWM_FREQ 500
#define PWM_RESOLUTION 8

//!---------------------       Cabeçalho de Funções     ---------------------

void connectToWIFI();

void callback(char *topic, byte *message, unsigned int length);
void connectToBroker();
void statusLED(byte status);
void turnOffLEDs();

//!---------------------       Definições de Constantes     ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);

//Values set in /include/env.h

const char *WIFI_SSID = WIFI_SSID;
const char *WIFI_PASSWORD = WIFI_PASSWORD;

const char *MQTT_BROKER = MQTT_BROKER;

const char *MQTT_USER = MQTT_USER;
const char *MQTT_PASS = MQTT_PASS;

const char *MQTT_BROKER = MQTT_BROKER;
const int MQTT_PORT = MQTT_PORT;

const char *topic = "esp_motor/speed";

//!---------------------       Loops Principais        ---------------------

void setup()
{
    Serial.begin(115200);
    // As the free tier of HiveMQ does not allow generating a CA, it is necessary to disable certificate verification
    client.setInsecure();

    // H-Bridge
    ledcSetup(PWM_FORWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_BACKWARD, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FORWARD_DIRECTION_PIN, PWM_FORWARD);
    ledcAttachPin(BACKWARD_DIRECTION_PIN, PWM_BACKWARD);
    ledcWrite(PWM_FORWARD, 0);
    ledcWrite(PWM_BACKWARD, 0);
    digitalWrite(FORWARD_DIRECTION_PIN, LOW);
    digitalWrite(BACKWARD_DIRECTION_PIN, LOW);

    // Status LED
    pinMode(STATUS_LED_R, OUTPUT);
    pinMode(STATUS_LED_G, OUTPUT);
    pinMode(STATUS_LED_B, OUTPUT);
    turnOffLEDs();

    connectToWiFi();
    connectToMQTT();
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        connectToWIFI();
    }
    if (!mqttClient.connected())
    {
        connectToBroker();
    }
    mqttClient.loop();
}

/*-------------------------------------------------------------------------------*/

void connectToWiFi() {
    statusLED(1);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Conectando ao WiFi...");
    while (!WiFi.isConnected()) {
      delay(1000);
      Serial.print(".");
    }
    if (WiFi.isConnected()) {
      Serial.println("Conectado ao WiFi!");
    } else {
      Serial.println("Falha ao conectar ao WiFi!");
      statusLED(-1);
    }
}

void connectToMQTT() {
    statusLED(2);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  
    while (!mqttClient.connected()) {
      Serial.print("Conectando ao Broker MQTT...");
      if (mqttClient.connect(MQTT_ID, MQTT_USER, MQTT_PASS)) {
        mqttClient.subscribe(topic);
        mqttClient.setCallback(callback);
  
        Serial.println("Conectado ao Broker MQTT");
        Serial.println("Inscrito no tópico: topico/teste");
      } else {
        statusLED(-1);
        Serial.println("Falha ao conectar ao Broker MQTT");
        Serial.print("Erro: ");
        Serial.println(mqttClient.state());
        delay(2000);
        Serial.println("Tentando novamente...");
      }
    }
  }

void statusLED(byte status){   
    turnOffLEDs();
    switch (status)    {
    case -1: // ERROR
        for (int i = 0; i < 4; i++){
            digitalWrite(STATUS_LED_R,!digitalRead(STATUS_LED_R));
            delay(100);
        }
        break;
    case 1: // WIFI_CONNECTION
        digitalWrite(STATUS_LED_R,HIGH);
        digitalWrite(STATUS_LED_G,HIGH);
        break;
    case 2: // MQTT_CONNECTION
        digitalWrite(STATUS_LED_R,HIGH);
        digitalWrite(STATUS_LED_B,HIGH);
        break;
    

    default:
        for (int i = 0; i < 4; i++){
            digitalWrite(STATUS_LED_B,!digitalRead(STATUS_LED_B));
            delay(100);
        }
        break;
    }
}

void turnOffLEDs(){
    digitalWrite(STATUS_LED_R, LOW);
    digitalWrite(STATUS_LED_G, LOW);
    digitalWrite(STATUS_LED_B, LOW);
}



void callback(char *topic, byte *payload, unsigned int length)
{
    String message = "";

    for (int i = 0; i < length; i++)
    {
        char c = (char)payload[i];
        if (!isDigit(c))
        {
            mqttClient.publish("esp_motor/status", "Valor invalido");
            return;
        }
        message += c;
    }

    int speed = message.toInt();
    if (speed > 0 && speed < 255)
    {
        ledcWrite(PWM_FORWARD, speed);
    }
    else
    {
        mqttClient.publish("esp_motor/status", "Valor invalido");
    }
}