//!---------------------       Inclusões de bibliotecas ---------------------

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "PubSubClient.h"
#include "env.h"
#include "topics.h"

//!---------------------       Definição dos pinos      ---------------------

#define PWM_FREQ 500
#define PWM_RESOLUTION 8

#define STATUS_LED_R_PIN 25
#define STATUS_LED_G_PIN 26
#define STATUS_LED_B_PIN 27

#define PWM_CHANNEL_LED_R 2
#define PWM_CHANNEL_LED_G 3
#define PWM_CHANNEL_LED_B 4



#define FORWARD_DIRECTION_PIN 32   
#define BACKWARD_DIRECTION_PIN 33  

#define PWM_FORWARD 0
#define PWM_BACKWARD 1


//!---------------------       Cabeçalho de Funções     ---------------------

void callback(char* topic, byte* message, unsigned int length);
void connectToMQTT();
void connectToWiFi();
void statusLED(byte status);
void turnOffLEDs();
void handleError();
void setSpeed(int speed);

//!---------------------       Definições de Constantes ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);



// Values set in /include/env.h

const char* mqtt_broker = MQTT_BROKER_CONN;
const char* mqtt_user = MQTT_USER_CONN;
const char* mqtt_password = MQTT_PASSWORD_CONN;
const int mqtt_port = MQTT_PORT_CONN;

const char* wifi_ssid = WIFI_CONN_SSID;
const char* wifi_password = WIFI_CONN_PASSWORD;

int currentSpeed = 0;

//!---------------------       Loops Principais        ---------------------

void setup() {
    Serial.begin(115200);
    // As the free tier of HiveMQ does not allow generating a CA, it is necessary
    // to disable certificate verification
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

    ledcSetup(PWM_CHANNEL_LED_R, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_G, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LED_B, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(STATUS_LED_R_PIN, PWM_CHANNEL_LED_R);
    ledcAttachPin(STATUS_LED_G_PIN, PWM_CHANNEL_LED_G);
    ledcAttachPin(STATUS_LED_B_PIN, PWM_CHANNEL_LED_B);
    turnOffLEDs();
    delay(2000);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        handleError();
        connectToWiFi();
    }
    if (!mqttClient.connected()) {
        handleError();
        connectToMQTT();
    }
    mqttClient.loop();
}

//!---------------------       Funções extras        ---------------------

void setLEDColor(byte r, byte g, byte b) {
    ledcWrite(PWM_CHANNEL_LED_R, r);
    ledcWrite(PWM_CHANNEL_LED_G, g);
    ledcWrite(PWM_CHANNEL_LED_B, b);
}

void connectToWiFi() {
    statusLED(1);
    delay(500);
    WiFi.begin(WIFI_CONN_SSID, WIFI_CONN_PASSWORD);
    Serial.print("Conectando ao WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Conectando ao WiFi: ");
        Serial.println(String(WIFI_CONN_SSID) + " / " + String(WIFI_CONN_PASSWORD));
    }
    else {
        Serial.println("Falha ao conectar ao WiFi!");
    }
}

void connectToMQTT() {
    statusLED(2);
    delay(500);
    mqttClient.setServer(MQTT_BROKER_CONN, MQTT_PORT_CONN);

    while (!mqttClient.connected()) {
        Serial.print("Conectando ao Broker MQTT...");

        String NODE_ID = "NODE_TRAIN-";
        NODE_ID += String(random(0xffff), HEX);
        if (mqttClient.connect(NODE_ID.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
            mqttClient.subscribe(topicTrainSpeed);
            mqttClient.setCallback(callback);

            Serial.println("Conectado ao Broker MQTT");
            Serial.print("Inscrito no tópico: ");
            Serial.print(topicTrainSpeed);
            turnOffLEDs();
        }
        else {
            Serial.println("Falha ao conectar ao Broker MQTT");
            Serial.print("Erro: ");
            Serial.println(mqttClient.state());
            delay(2000);
            Serial.println("Tentando novamente...");
        }
    }
}

void statusLED(byte status) {
    turnOffLEDs();
    switch (status) {
    case 254:  // Erro (Vermelho)
        setLEDColor(255, 0, 0);
        break;

    case 1:  // Conectando ao Wi-Fi (Amarelo)
        setLEDColor(150, 255, 0);
        break;

    case 2:  // Conectando ao MQTT (Rosa)
        setLEDColor(150, 0, 255);
        break;

    case 3:  // Movendo para frente (Verde)
        setLEDColor(0, 255, 0);
        break;

    case 4:  // Movendo para trás (Ciano)
        setLEDColor(0, 255, 255);
        break;

    default:
        for (byte i = 0; i < 4; i++) {
            setLEDColor(0, 0, 255);  // erro no status (pisca azul)
            delay(100);
            turnOffLEDs();
            delay(100);
        }
        break;
    }
}

void turnOffLEDs() { setLEDColor(0, 0, 0); }

void handleError() {
    for (byte i = 0; i < 4; i++) {
        statusLED(254);
        delay(100);
        turnOffLEDs();
        delay(100);
    }
    turnOffLEDs();
    mqttClient.publish(topicTrainStatus, "Valor invalido");
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";

    for (int i = 0; i < length; i++) {
        char c = (char)payload[i];
        if (!isDigit(c)) {
            handleError();
            return;
        }
        message += c;
    }

    int speed = message.toInt();
    if (speed > -255 && speed < 255) {
        if (speed != currentSpeed) {
            currentSpeed = speed;
            setSpeed(speed);
            Serial.println(String("Velocidade alterada para: ") + speed);
        }
    }
  else {
      handleError();
      statusLED(3);
  }
}

void setSpeed(int speed){
    ledcWrite(PWM_FORWARD, 0);
    ledcWrite(PWM_BACKWARD, 0);
    delay(500);
    if(speed > 0){
        statusLED(3);
        for(int i = 0; i <= speed; i++){
            ledcWrite(PWM_FORWARD, i);
            delay(5);
        }
    }else if (speed < 0){
        statusLED(4);
        for (int i = 0; i >= speed; i--) {
            ledcWrite(PWM_BACKWARD, -i);
            delay(5);
        }
    }else{
        turnOffLEDs();
        ledcWrite(PWM_FORWARD, 0);
        ledcWrite(PWM_BACKWARD, 0);
    }
}