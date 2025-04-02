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

void callback(char *topic, byte *message, unsigned int length);
void connectToMQTT();
void connectToWiFi();
void statusLED(byte status);
void turnOffLEDs();
void handleError();

//!---------------------       Definições de Constantes     ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);

//Values set in /include/env.h

const char *mqtt_broker = MQTT_BROKER_CONN;
const char *mqtt_user = MQTT_USER_CONN;
const char *mqtt_password = MQTT_PASSWORD_CONN;
const char *mqtt_id = MQTT_ID_CONN;
const int mqtt_port = MQTT_PORT_CONN;

const char *wifi_ssid = WIFI_CONN_SSID;
const char *wifi_password = WIFI_CONN_PASSWORD;

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
    delay(2000);
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        handleError();
        connectToWiFi();
    }
    if (!mqttClient.connected())
    {
        handleError();
        connectToMQTT();
    }
    mqttClient.loop();
}

//!---------------------       Funções extras        ---------------------

void connectToWiFi() {
    statusLED(1);
    delay(500);
    WiFi.begin(wifi_ssid, wifi_password);
    Serial.print("Conectando ao WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Conectando ao WiFi: ");
        Serial.println(String(wifi_ssid) + " / " + String(wifi_password));
    } else {
      Serial.println("Falha ao conectar ao WiFi!");
      
    }
}

void connectToMQTT() {
    statusLED(2);
    delay(500);
    mqttClient.setServer(mqtt_broker, mqtt_port);
  
    while (!mqttClient.connected()) {
      Serial.print("Conectando ao Broker MQTT...");
      if (mqttClient.connect(mqtt_id, mqtt_user, mqtt_password)) {
        mqttClient.subscribe(topic);
        mqttClient.setCallback(callback);
  
        Serial.println("Conectado ao Broker MQTT");
        Serial.print("Inscrito no tópico: ");
        Serial.print(topic);
        turnOffLEDs();
      } else {
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
    case 254: // ERROR       RED
        digitalWrite(STATUS_LED_R,HIGH);
        break;

    case 1: // WIFI_CONNECTION      YELLOW
        digitalWrite(STATUS_LED_G,HIGH);
        break;

    case 2: // MQTT_CONNECTION      PINK
        digitalWrite(STATUS_LED_B,HIGH);
        break;

    case 3: //FORWARD               GREEN
        digitalWrite(STATUS_LED_G,HIGH);
        break;

    case 4: //BACKWARD              LIGHT BLUE
        digitalWrite(STATUS_LED_R,HIGH);
        break;

    default:
        for (byte i = 0; i < 4; i++){
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

void handleError(){
    for(byte i = 0; i < 4; i++){
        statusLED(254);
        delay(100);
        turnOffLEDs();
        delay(100);
    }
    turnOffLEDs();
    mqttClient.publish("esp_motor/status", "Valor invalido");
}


void callback(char *topic, byte *payload, unsigned int length)
{
    String message = "";

    for (int i = 0; i < length; i++)
    {
        char c = (char)payload[i];
        if (!isDigit(c))
        {
            handleError();
            return;
        }
        message += c;
    }

    int speed = message.toInt();
    if (speed >= 0 && speed < 255)
    {
        statusLED(3);
        ledcWrite(PWM_FORWARD, speed);
        Serial.println(String("Velocidade alterada para: ") + speed);
    }
    else
    {   
        statusLED(3);
        handleError();
    }
}